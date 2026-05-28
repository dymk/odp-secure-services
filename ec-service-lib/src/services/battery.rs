//! `Battery` — FFA service that proxies Normal-World `GetBst` requests
//! to the EC's `BatteryServiceRelayHandler` over MCTP.
//!
//! Transport-agnostic via the [`odp_client::Relay`] trait. [`Battery`]
//! is generic over `R: Relay`; the concrete relay impl (e.g.
//! `OdpClient<SerialTransport<Pl011Uart>>` in production, or
//! `OdpClient<LoopbackTransport>` in tests) is inferred at the wiring
//! call site. Battery itself never names a transport type.
//!
//! # Wire format (must match the EC's `OdpRelayHandler` byte-for-byte)
//!
//! Battery service id = `0x08`; `BatteryCmd::GetBst` = 2.
//!
//! Request body (5 bytes, post the MCTP `MESSAGE_TYPE = 0x7D` byte):
//! ```text
//!     [0x02, 0x08, 0x00, 0x02, 0x00]
//!      \________________/  ^
//!       OdpHeader (BE u32)  battery_id = 0
//!       (req=1, svc=0x08,
//!        is_error=0, msg_id=2)
//! ```
//!
//! Response body (20 bytes): 4-byte BE OdpHeader (req=0, svc=0x08, msg_id=2)
//! followed by 16 bytes (4 LE u32 dwords: `battery_state.bits()`,
//! `battery_present_rate`, `battery_remaining_capacity`,
//! `battery_present_voltage`).
//!
//! # SP-side runtime serialization is manual
//!
//! `odp_client::SerializableMessage` (re-exported from
//! `odp_client::serializable`) does not compile for
//! `aarch64-unknown-none-softfloat` (the SBSA SP target) because
//! `embassy-sync::ThreadModeRawMutex` is `cortex_m`-gated in the
//! `battery-service-relay` types it would deserialize into. As a
//! workaround, SP-side runtime serialization for `GetBst` is performed
//! MANUALLY (1-byte request payload; 4 LE u32 dwords for the response
//! body). The wire-format gate test in the `tests` module below
//! round-trips bytes through the EC's OWN `SerializableMessage` impl
//! (via `[dev-dependencies]`) so any drift fails the build.

use core::cell::RefCell;

use uuid::{uuid, Uuid};

use crate::{Result, Service};
use odp_client::{OdpError, OdpHeader, OdpService, Relay};
use odp_ffa::{Error as FfaError, MsgSendDirectReq2, MsgSendDirectResp2};

/// `BatteryCmd::GetBst` discriminant from
/// `embedded-services/battery-service-relay/src/serialization.rs`.
pub const BATTERY_CMD_GET_BST: u16 = 2;

/// Body of a `GetBst` response, post-OdpHeader. 16 bytes (4 LE u32 dwords).
pub const GET_BST_RESPONSE_BODY_LEN: usize = 16;

/// Parsed `GetBst` response (mirrors
/// `battery_service_interface::BstReturn` field-for-field but stays
/// dependency-free at runtime).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BstReturnRaw {
    pub battery_state: u32,
    pub battery_present_rate: u32,
    pub battery_remaining_capacity: u32,
    pub battery_present_voltage: u32,
}

/// Errors returned by [`Battery::get_bst`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryError {
    /// Relay (MCTP + transport) failure — see [`OdpError`].
    Relay(OdpError),
    /// EC's response was not a Battery service response or not the
    /// expected message id.
    UnexpectedResponse,
}

impl From<OdpError> for BatteryError {
    fn from(e: OdpError) -> Self {
        BatteryError::Relay(e)
    }
}

/// `Battery` holds a handle to a shared [`Relay`] — it owns no
/// transport, no assembly buffer, no MCTP framing state. Construction
/// takes only a borrow of the wiring-layer-owned `RefCell<R>`.
///
/// Battery is generic over `R: Relay` rather than over a transport
/// type: it never names `OdpTransport`, `SerialTransport`, or any
/// UART type. The concrete `R = OdpClient<T>` is inferred at the
/// wiring call site. This keeps transport-layer details out of the
/// service layer's API.
///
/// Future EC-proxy services (`EcThermal`, `EcFwMgmt`, `EcTimeAlarm`)
/// follow the same pattern: `Self::new(relay: &'r RefCell<R>)`. They
/// all share the single physical EC channel by borrowing the same
/// `RefCell`-wrapped relay.
pub struct Battery<'r, R: Relay> {
    relay: &'r RefCell<R>,
}

impl<'r, R: Relay> Battery<'r, R> {
    pub fn new(relay: &'r RefCell<R>) -> Self {
        Self { relay }
    }

    /// Drive a single GetBst request/response round-trip over the EC
    /// MCTP relay. Returns the parsed BST body or a relay/wire-format
    /// error.
    pub fn get_bst(&self, battery_id: u8) -> core::result::Result<BstReturnRaw, BatteryError> {
        let request_header = OdpHeader {
            is_request: true,
            service: OdpService::Battery,
            is_error: false,
            message_id: BATTERY_CMD_GET_BST,
        };
        let request_body = [battery_id];

        let mut relay = self.relay.borrow_mut();
        let response = relay
            .invoke(request_header, &request_body)
            .map_err(BatteryError::Relay)?;
        // `OdpClient::invoke` already validated the message_id round-trip;
        // we only need to confirm the response is for the Battery service.
        if response.header.service != OdpService::Battery {
            return Err(BatteryError::Relay(OdpError::UnexpectedResponseKind));
        }
        if response.body.len() < GET_BST_RESPONSE_BODY_LEN {
            return Err(BatteryError::Relay(OdpError::Decode));
        }
        let payload = &response.body[..GET_BST_RESPONSE_BODY_LEN];
        Ok(BstReturnRaw {
            battery_state: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            battery_present_rate: u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            battery_remaining_capacity: u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            battery_present_voltage: u32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]),
        })
    }
}

impl<R: Relay> Service for Battery<'_, R> {
    const UUID: Uuid = uuid!("25cb5207-ac36-427d-aaef-3aa78877d27e");
    const NAME: &'static str = "Battery";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        // The EFI test-app sends a single GetBst with battery_id = 0
        // (UEFI parses no payload bytes for this round-trip). Future
        // callers may extract `battery_id` from msg.payload().u8_at(0).
        match self.get_bst(0) {
            Ok(bst) => {
                // Pack the 16 BST bytes as 4 LE u32 dwords across the
                // direct-message register payload (mirrors notify.rs's
                // `From<NfyGenericRsp>` pattern of placing scalar
                // values at the start of the payload).
                let payload = odp_ffa::DirectMessagePayload::from_iter(
                    bst.battery_state
                        .to_le_bytes()
                        .into_iter()
                        .chain(bst.battery_present_rate.to_le_bytes())
                        .chain(bst.battery_remaining_capacity.to_le_bytes())
                        .chain(bst.battery_present_voltage.to_le_bytes()),
                );
                Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
            }
            Err(_) => Err(FfaError::Other(
                "Battery: GetBst round-trip failed (transport or wire decode)",
            )),
        }
    }
}

// ===========================================================================
// Wire-format compatibility gate
//
// Round-trips bytes through the EC's OWN `SerializableMessage::serialize` /
// `deserialize` impls from `embedded-services/battery-service-relay`. Any
// drift in field order, endianness, or discriminant numbering causes the
// assertion to fail. Runs on host (std available) — no QEMU.
// ===========================================================================

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use battery_service_interface::{BatteryState, BstReturn};
    use battery_service_relay::{AcpiBatteryRequest, AcpiBatteryResponse};
    use odp_client::{OdpClient, OdpTransport, SerializableMessage};
    use std::rc::Rc;
    use std::vec::Vec;

    /// Test-only `OdpTransport` impl: records bytes that `OdpClient`
    /// sends into a shared `Rc<RefCell<Vec<u8>>>` (so the test body can
    /// inspect the request wire bytes after the round-trip) and returns
    /// a pre-loaded canned ODP message on the next `recv_message`.
    /// Operates at the pure ODP message level (4-byte header + body);
    /// no MCTP framing — that's `SerialTransport`'s job in production,
    /// and is out of scope for service-layer tests.
    struct CannedResponseTransport {
        sent: Rc<RefCell<Vec<u8>>>,
        response: Vec<u8>,
    }

    impl OdpTransport for CannedResponseTransport {
        fn send_message(&mut self, payload: &[u8]) -> core::result::Result<(), OdpError> {
            self.sent.borrow_mut().extend_from_slice(payload);
            Ok(())
        }

        fn recv_message(&mut self, buf: &mut [u8]) -> core::result::Result<usize, OdpError> {
            if self.response.is_empty() {
                return Err(OdpError::Transport);
            }
            if buf.len() < self.response.len() {
                return Err(OdpError::BufferTooSmall);
            }
            let n = self.response.len();
            buf[..n].copy_from_slice(&self.response);
            self.response.clear();
            Ok(n)
        }
    }

    fn canned_bst() -> BstReturn {
        BstReturn {
            battery_state: BatteryState::from_bits_truncate(0x0000_0001), // discharging
            battery_present_rate: 0x1122_3344,
            battery_remaining_capacity: 0x5566_7788,
            battery_present_voltage: 0x99AA_BBCC,
        }
    }

    #[test]
    fn round_trips_get_bst_against_ec_serializer() {
        // -- Synthesize a response so Battery::get_bst doesn't block on transport read.
        //    Bytes are produced by the EC's OWN `SerializableMessage`
        //    impl: any drift in field order/endianness fails the assert.
        let bst = canned_bst();
        let mut response_body = [0u8; 16];
        let n = AcpiBatteryResponse::GetBst { bst }
            .serialize(&mut response_body)
            .expect("ec-side serialize");
        assert_eq!(n, 16, "GetBst response body must be 16 bytes");

        let response_header = OdpHeader {
            is_request: false,
            service: OdpService::Battery,
            is_error: false,
            message_id: BATTERY_CMD_GET_BST,
        };

        let mut canned = Vec::new();
        canned.extend_from_slice(&response_header.to_be_bytes());
        canned.extend_from_slice(&response_body);

        let sent = Rc::new(RefCell::new(Vec::new()));
        let transport = CannedResponseTransport {
            sent: Rc::clone(&sent),
            response: canned,
        };
        let relay = RefCell::new(OdpClient::new(transport));
        let svc = Battery::new(&relay);

        // -- Drive the round-trip.
        let result = svc.get_bst(0).expect("get_bst should decode synthesized response");

        // -- ASSERT: Battery returned the BST values synthesized at the top.
        //    The response wire-format gate is exercised: bytes are
        //    produced by the EC's `AcpiBatteryResponse::serialize` and
        //    must round-trip through Battery::get_bst's decoder.
        assert_eq!(result.battery_state, bst.battery_state.bits());
        assert_eq!(result.battery_present_rate, bst.battery_present_rate);
        assert_eq!(result.battery_remaining_capacity, bst.battery_remaining_capacity);
        assert_eq!(result.battery_present_voltage, bst.battery_present_voltage);

        // -- ASSERT: the request bytes that `odp-client` produced parse
        //    back to `AcpiBatteryRequest::GetBst { battery_id: 0 }` via
        //    the EC's OWN deserializer. This validates the SP→EC
        //    request wire format end-to-end without re-asserting raw
        //    bytes (raw-byte encoding is now owned and tested by
        //    `odp-client`).
        let sent = sent.borrow();
        assert!(sent.len() >= 4, "request must include 4-byte OdpHeader");
        let mut hdr_bytes = [0u8; 4];
        hdr_bytes.copy_from_slice(&sent[..4]);
        let parsed_hdr = OdpHeader::from_be_bytes(hdr_bytes).expect("parse header");
        assert!(parsed_hdr.is_request, "must be a request");
        assert_eq!(parsed_hdr.service, OdpService::Battery);
        assert_eq!(parsed_hdr.message_id, BATTERY_CMD_GET_BST);
        let decoded = AcpiBatteryRequest::deserialize(parsed_hdr.message_id, &sent[4..])
            .expect("ec-side decoder must accept SP-produced bytes");
        assert!(
            matches!(decoded, AcpiBatteryRequest::GetBst { battery_id: 0 }),
            "EC-side decoder must reconstruct the original request variant"
        );
    }
}
