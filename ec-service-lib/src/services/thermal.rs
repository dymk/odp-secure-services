use crate::service::Service;
use crate::Result;
use log::{debug, error};
use odp_ffa::{DirectMessagePayload, Function, HasRegisterPayload, MsgSendDirectReq2, MsgSendDirectResp2, Yield};
use uuid::{uuid, Builder, Uuid};

// Protocol CMD definitions for Thermal
const EC_THM_GET_TMP: u8 = 0x1;
const EC_THM_SET_THRS: u8 = 0x2;
const EC_THM_GET_THRS: u8 = 0x3;
const EC_THM_SET_SCP: u8 = 0x4;
const EC_THM_GET_VAR: u8 = 0x5;
const EC_THM_SET_VAR: u8 = 0x6;

const MAX_SENSORS: usize = 8;
const MAX_VARS: usize = 16;

// Status codes returned in the `status` field of Thermal responses.
const THM_STATUS_OK: i64 = 0;
const THM_STATUS_ERR: i64 = -1;

#[derive(Default)]
struct GenericRsp {
    status: i64,
}

impl From<GenericRsp> for DirectMessagePayload {
    fn from(value: GenericRsp) -> Self {
        DirectMessagePayload::from_iter(value.status.to_le_bytes())
    }
}

#[derive(Default)]
struct TempRsp {
    status: i64,
    temp: u64,
}

impl From<TempRsp> for DirectMessagePayload {
    fn from(value: TempRsp) -> Self {
        DirectMessagePayload::from_iter(value.status.to_le_bytes().into_iter().chain(value.temp.to_le_bytes()))
    }
}

#[derive(Default)]
struct ThresholdReq {
    id: u8,
    timeout: u32,
    low_temp: u32,
    high_temp: u32,
}
impl From<&DirectMessagePayload> for ThresholdReq {
    fn from(msg: &DirectMessagePayload) -> ThresholdReq {
        ThresholdReq {
            id: msg.u8_at(1),
            timeout: msg.u16_at(3) as u32,
            low_temp: msg.u32_at(5),
            high_temp: msg.u32_at(9),
        }
    }
}

#[derive(Default)]
struct ReadVarReq {
    id: u8,
    len: u16,
    var_uuid: Uuid,
}

impl From<ReadVarReq> for DirectMessagePayload {
    fn from(value: ReadVarReq) -> Self {
        let iter = value
            .id
            .to_le_bytes()
            .into_iter()
            .chain(value.len.to_le_bytes())
            .chain(value.var_uuid.as_bytes().iter().copied());

        DirectMessagePayload::from_iter(iter)
    }
}

impl From<&DirectMessagePayload> for ReadVarReq {
    fn from(msg: &DirectMessagePayload) -> ReadVarReq {
        ReadVarReq {
            id: msg.u8_at(1),
            len: msg.u16_at(2),
            var_uuid: Builder::from_slice_le(msg.slice(4..20)).unwrap().into_uuid(),
        }
    }
}

#[derive(Default)]
struct ReadVarRsp {
    status: i64,
    data: u32,
}

impl From<ReadVarRsp> for DirectMessagePayload {
    fn from(value: ReadVarRsp) -> Self {
        DirectMessagePayload::from_iter(value.status.to_le_bytes().into_iter().chain(value.data.to_le_bytes()))
    }
}

#[derive(Default)]
struct SetVarReq {
    id: u8,
    len: u16,
    var_uuid: Uuid,
    data: u32,
}

impl From<&DirectMessagePayload> for SetVarReq {
    fn from(msg: &DirectMessagePayload) -> SetVarReq {
        SetVarReq {
            id: msg.u8_at(1),
            len: msg.u16_at(2),
            var_uuid: Builder::from_slice_le(msg.slice(4..20)).unwrap().into_uuid(),
            data: msg.u32_at(20),
        }
    }
}

#[derive(Default, Clone, Copy)]
struct ThresholdData {
    timeout: u32,
    low_temp: u32,
    high_temp: u32,
}

#[derive(Default)]
struct ThresholdRsp {
    status: i64,
    timeout: u32,
    low_temp: u32,
    high_temp: u32,
}

impl From<ThresholdRsp> for DirectMessagePayload {
    fn from(value: ThresholdRsp) -> Self {
        DirectMessagePayload::from_iter(
            value
                .status
                .to_le_bytes()
                .into_iter()
                .chain(value.timeout.to_le_bytes())
                .chain(value.low_temp.to_le_bytes())
                .chain(value.high_temp.to_le_bytes()),
        )
    }
}

struct CoolingPolicyReq {
    id: u8,
    policy_type: u8,
}

impl From<&DirectMessagePayload> for CoolingPolicyReq {
    fn from(msg: &DirectMessagePayload) -> CoolingPolicyReq {
        CoolingPolicyReq {
            id: msg.u8_at(1),
            policy_type: msg.u8_at(2),
        }
    }
}

pub struct Thermal {
    thresholds: [ThresholdData; MAX_SENSORS],
    variables: [(Uuid, u32); MAX_VARS],
    var_count: usize,
}

impl Default for Thermal {
    fn default() -> Self {
        Self::new()
    }
}

impl Thermal {
    pub fn new() -> Self {
        Thermal {
            thresholds: [ThresholdData::default(); MAX_SENSORS],
            variables: [(Uuid::nil(), 0u32); MAX_VARS],
            var_count: 0,
        }
    }

    // Validates a sensor id and returns its array index, or None if out of range.
    fn sensor_index(id: u8) -> Option<usize> {
        let idx = id as usize;
        (idx < MAX_SENSORS).then_some(idx)
    }

    // Linear scan for a stored variable by UUID.
    fn find_var(&self, var_uuid: &Uuid) -> Option<usize> {
        (0..self.var_count).find(|&i| self.variables[i].0 == *var_uuid)
    }

    fn get_temperature(&mut self, msg: &MsgSendDirectReq2) -> TempRsp {
        debug!("get_temperature sensor 0x{:x}", msg.payload().u8_at(1));

        // Tell OS to delay 1 ms
        Yield::new(0x100000000).exec().unwrap();

        TempRsp {
            status: THM_STATUS_OK,
            temp: 0x1234,
        }
    }

    fn set_threshold(&mut self, msg: &MsgSendDirectReq2) -> GenericRsp {
        let req: ThresholdReq = msg.payload().into();
        debug!(
            "set_threshold sensor 0x{:x} timeout=0x{:x} low=0x{:x} high=0x{:x}",
            req.id, req.timeout, req.low_temp, req.high_temp
        );

        let Some(idx) = Self::sensor_index(req.id) else {
            return GenericRsp { status: THM_STATUS_ERR };
        };

        self.thresholds[idx] = ThresholdData {
            timeout: req.timeout,
            low_temp: req.low_temp,
            high_temp: req.high_temp,
        };

        GenericRsp { status: THM_STATUS_OK }
    }

    fn get_threshold(&self, msg: &MsgSendDirectReq2) -> ThresholdRsp {
        let id = msg.payload().u8_at(1);

        let Some(idx) = Self::sensor_index(id) else {
            return ThresholdRsp {
                status: THM_STATUS_ERR,
                ..Default::default()
            };
        };

        // Unset sensors read back as zeroed defaults.
        let data = self.thresholds[idx];
        ThresholdRsp {
            status: THM_STATUS_OK,
            timeout: data.timeout,
            low_temp: data.low_temp,
            high_temp: data.high_temp,
        }
    }

    fn set_cooling_policy(&mut self, msg: &MsgSendDirectReq2) -> GenericRsp {
        let req: CoolingPolicyReq = msg.payload().into();
        debug!(
            "set_cooling_policy sensor 0x{:x} policy=0x{:x}",
            req.id, req.policy_type
        );

        // Validate the sensor id. The policy is accepted but not stored — there
        // is no GET command that reads it back.
        if Self::sensor_index(req.id).is_none() {
            return GenericRsp { status: THM_STATUS_ERR };
        }

        GenericRsp { status: THM_STATUS_OK }
    }

    fn get_variable(&self, msg: &MsgSendDirectReq2) -> ReadVarRsp {
        let req: ReadVarReq = msg.payload().into();
        debug!(
            "get_variable instance id: 0x{:x} length: 0x{:x} uuid: {}",
            req.id, req.len, req.var_uuid
        );

        if req.len != 4 {
            error!("get_variable only supports DWORD read");
            return ReadVarRsp {
                status: THM_STATUS_ERR,
                data: 0,
            };
        }

        if let Some(i) = self.find_var(&req.var_uuid) {
            return ReadVarRsp {
                status: THM_STATUS_OK,
                data: self.variables[i].1,
            };
        }

        // UUID not found
        ReadVarRsp {
            status: THM_STATUS_ERR,
            data: 0,
        }
    }

    fn set_variable(&mut self, msg: &MsgSendDirectReq2) -> GenericRsp {
        let req: SetVarReq = msg.payload().into();
        debug!(
            "set_variable instance id: 0x{:x} length: 0x{:x} uuid: {} data: 0x{:x}",
            req.id, req.len, req.var_uuid, req.data
        );

        if req.len != 4 {
            error!("set_variable only supports DWORD write");
            return GenericRsp { status: THM_STATUS_ERR };
        }

        // Upsert: update in place if the UUID already exists.
        if let Some(i) = self.find_var(&req.var_uuid) {
            self.variables[i].1 = req.data;
            return GenericRsp { status: THM_STATUS_OK };
        }

        // Otherwise append to an open slot; reject if the store is full.
        if self.var_count >= MAX_VARS {
            error!("set_variable: variable store full");
            return GenericRsp { status: THM_STATUS_ERR };
        }
        self.variables[self.var_count] = (req.var_uuid, req.data);
        self.var_count += 1;

        GenericRsp { status: THM_STATUS_OK }
    }
}

impl Service for Thermal {
    const UUID: Uuid = uuid!("31f56da7-593c-4d72-a4b3-8fc7171ac073");
    const NAME: &'static str = "Thermal";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let cmd = msg.payload().u8_at(0);
        debug!("Received ThmMgmt command 0x{:x}", cmd);

        let payload = match cmd {
            EC_THM_GET_TMP => DirectMessagePayload::from(self.get_temperature(&msg)),
            EC_THM_SET_THRS => DirectMessagePayload::from(self.set_threshold(&msg)),
            EC_THM_GET_THRS => DirectMessagePayload::from(self.get_threshold(&msg)),
            EC_THM_SET_SCP => DirectMessagePayload::from(self.set_cooling_policy(&msg)),
            EC_THM_GET_VAR => DirectMessagePayload::from(self.get_variable(&msg)),
            EC_THM_SET_VAR => DirectMessagePayload::from(self.set_variable(&msg)),
            _ => {
                error!("Unknown Thermal Command: {}", cmd);
                return Err(odp_ffa::Error::Other("Unknown Thermal Command"));
            }
        };

        Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use odp_ffa::{DirectMessagePayload, HasRegisterPayload, MsgSendDirectReq2};
    use uuid::uuid;

    const THERMAL_UUID: Uuid = uuid!("31f56da7-593c-4d72-a4b3-8fc7171ac073");

    // ===================================================================
    // Helpers
    // ===================================================================

    /// Wrap a payload into a MsgSendDirectReq2 addressed to the Thermal service.
    fn thermal_req(payload: DirectMessagePayload) -> MsgSendDirectReq2 {
        MsgSendDirectReq2::new(0, 0, THERMAL_UUID, payload)
    }

    /// Extract status (i64) from response payload bytes 0-7.
    fn resp_status(resp: &MsgSendDirectResp2) -> i64 {
        resp.payload().u64_at(0) as i64
    }

    /// Build a set_threshold request payload.
    /// Layout: byte 0=EC_THM_SET_THRS(0x2), byte 1=sensor_id, bytes 3-4=timeout(u16 LE),
    ///         bytes 5-8=low_temp(u32 LE), bytes 9-12=high_temp(u32 LE).
    fn set_threshold_payload(sensor_id: u8, timeout: u16, low_temp: u32, high_temp: u32) -> DirectMessagePayload {
        let mut bytes = [0u8; 14 * 8];
        bytes[0] = EC_THM_SET_THRS;
        bytes[1] = sensor_id;
        bytes[3..5].copy_from_slice(&timeout.to_le_bytes());
        bytes[5..9].copy_from_slice(&low_temp.to_le_bytes());
        bytes[9..13].copy_from_slice(&high_temp.to_le_bytes());
        DirectMessagePayload::from_iter(bytes)
    }

    /// Build a get_threshold request payload.
    /// Layout: byte 0=EC_THM_GET_THRS(0x3), byte 1=sensor_id.
    fn get_threshold_payload(sensor_id: u8) -> DirectMessagePayload {
        let mut bytes = [0u8; 14 * 8];
        bytes[0] = EC_THM_GET_THRS;
        bytes[1] = sensor_id;
        DirectMessagePayload::from_iter(bytes)
    }

    /// Build a set_cooling_policy request payload.
    /// Layout: byte 0=EC_THM_SET_SCP(0x4), byte 1=sensor_id, byte 2=policy_type.
    fn set_cooling_policy_payload(sensor_id: u8, policy_type: u8) -> DirectMessagePayload {
        let mut bytes = [0u8; 14 * 8];
        bytes[0] = EC_THM_SET_SCP;
        bytes[1] = sensor_id;
        bytes[2] = policy_type;
        DirectMessagePayload::from_iter(bytes)
    }

    /// Build a set_variable request payload.
    /// Layout: byte 0=EC_THM_SET_VAR(0x6), byte 1=instance_id, bytes 2-3=length(u16 LE),
    ///         bytes 4-19=UUID(16 bytes LE), bytes 20-23=data(u32 LE).
    fn set_variable_payload(uuid: &Uuid, data: u32) -> DirectMessagePayload {
        let mut bytes = [0u8; 14 * 8];
        bytes[0] = EC_THM_SET_VAR;
        bytes[1] = 0; // instance_id
        bytes[2..4].copy_from_slice(&4u16.to_le_bytes()); // len = 4 (DWORD)
        bytes[4..20].copy_from_slice(&uuid.to_bytes_le());
        bytes[20..24].copy_from_slice(&data.to_le_bytes());
        DirectMessagePayload::from_iter(bytes)
    }

    /// Build a get_variable request payload.
    /// Layout: byte 0=EC_THM_GET_VAR(0x5), byte 1=instance_id, bytes 2-3=length(u16 LE),
    ///         bytes 4-19=UUID(16 bytes LE).
    fn get_variable_payload(uuid: &Uuid) -> DirectMessagePayload {
        let mut bytes = [0u8; 14 * 8];
        bytes[0] = EC_THM_GET_VAR;
        bytes[1] = 0; // instance_id
        bytes[2..4].copy_from_slice(&4u16.to_le_bytes()); // len = 4 (DWORD)
        bytes[4..20].copy_from_slice(&uuid.to_bytes_le());
        DirectMessagePayload::from_iter(bytes)
    }

    // ===================================================================
    // Core Round-Trip Tests
    // ===================================================================

    #[test]
    fn test_set_get_threshold_round_trip() {
        let mut thermal = Thermal::new();
        let sensor_id = 2u8;
        let timeout = 500u16;
        let low_temp = 15u32;
        let high_temp = 85u32;

        // Set threshold
        let set_msg = thermal_req(set_threshold_payload(sensor_id, timeout, low_temp, high_temp));
        let set_resp = thermal.ffa_msg_send_direct_req2(set_msg).unwrap();
        assert_eq!(resp_status(&set_resp), 0);

        // Get threshold — assert round-trip
        let get_msg = thermal_req(get_threshold_payload(sensor_id));
        let get_resp = thermal.ffa_msg_send_direct_req2(get_msg).unwrap();
        let p = get_resp.payload();
        assert_eq!(p.u64_at(0) as i64, 0, "status should be 0");
        assert_eq!(p.u32_at(8), timeout as u32, "timeout round-trip");
        assert_eq!(p.u32_at(12), low_temp, "low_temp round-trip");
        assert_eq!(p.u32_at(16), high_temp, "high_temp round-trip");
    }

    #[test]
    fn test_set_get_variable_round_trip() {
        let mut thermal = Thermal::new();
        let var_uuid = uuid!("aabbccdd-1122-3344-5566-778899aabbcc");
        let data = 0x42u32;

        // Set variable
        let set_msg = thermal_req(set_variable_payload(&var_uuid, data));
        let set_resp = thermal.ffa_msg_send_direct_req2(set_msg).unwrap();
        assert_eq!(resp_status(&set_resp), 0);

        // Get variable — assert round-trip
        let get_msg = thermal_req(get_variable_payload(&var_uuid));
        let get_resp = thermal.ffa_msg_send_direct_req2(get_msg).unwrap();
        let p = get_resp.payload();
        assert_eq!(p.u64_at(0) as i64, 0, "status should be 0");
        assert_eq!(p.u32_at(8), data, "data round-trip");
    }

    #[test]
    fn test_set_cooling_policy_returns_success() {
        let mut thermal = Thermal::new();

        let msg = thermal_req(set_cooling_policy_payload(0, 1));
        let resp = thermal.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_status(&resp), 0);
    }

    /// Verifies TempRsp serialization matches the expected payload layout.
    /// Note: Cannot test `get_temperature` through ffa_msg_send_direct_req2 dispatch
    /// because `Yield::exec()` calls `ffa_smc` which panics when odp-ffa is compiled
    /// without cfg(test) (it's a dependency, not the crate under test).
    #[test]
    fn test_get_temperature_response_format() {
        let rsp = TempRsp {
            status: 0,
            temp: 0x1234,
        };
        let payload = DirectMessagePayload::from(rsp);
        assert_eq!(payload.u64_at(0) as i64, 0, "status should be 0");
        assert_eq!(payload.u64_at(8), 0x1234, "temperature should be 0x1234");
    }

    // ===================================================================
    // Edge Case Tests
    // ===================================================================

    #[test]
    fn test_get_threshold_unset_sensor_returns_zeroed_defaults() {
        let mut thermal = Thermal::new();

        // Query sensor 0 without setting anything
        let msg = thermal_req(get_threshold_payload(0));
        let resp = thermal.ffa_msg_send_direct_req2(msg).unwrap();
        let p = resp.payload();
        assert_eq!(p.u64_at(0) as i64, 0, "status should be 0 for unset sensor");
        assert_eq!(p.u32_at(8), 0, "timeout should be 0 for unset sensor");
        assert_eq!(p.u32_at(12), 0, "low_temp should be 0 for unset sensor");
        assert_eq!(p.u32_at(16), 0, "high_temp should be 0 for unset sensor");
    }

    #[test]
    fn test_get_variable_unknown_uuid_returns_error() {
        let mut thermal = Thermal::new();
        let unknown_uuid = uuid!("00000000-0000-0000-0000-000000000001");

        let msg = thermal_req(get_variable_payload(&unknown_uuid));
        let resp = thermal.ffa_msg_send_direct_req2(msg).unwrap();
        let p = resp.payload();
        assert_eq!(p.u64_at(0) as i64, -1, "status should be -1 for unknown UUID");
        assert_eq!(p.u32_at(8), 0, "data should be 0 for unknown UUID");
    }

    #[test]
    fn test_set_threshold_upsert_overwrites() {
        let mut thermal = Thermal::new();
        let sensor_id = 1u8;

        // Set initial values
        let msg1 = thermal_req(set_threshold_payload(sensor_id, 100, 10, 50));
        thermal.ffa_msg_send_direct_req2(msg1).unwrap();

        // Overwrite with new values
        let msg2 = thermal_req(set_threshold_payload(sensor_id, 200, 20, 90));
        thermal.ffa_msg_send_direct_req2(msg2).unwrap();

        // Get should return the latest values
        let get_msg = thermal_req(get_threshold_payload(sensor_id));
        let resp = thermal.ffa_msg_send_direct_req2(get_msg).unwrap();
        let p = resp.payload();
        assert_eq!(p.u64_at(0) as i64, 0);
        assert_eq!(p.u32_at(8), 200, "timeout should be updated value");
        assert_eq!(p.u32_at(12), 20, "low_temp should be updated value");
        assert_eq!(p.u32_at(16), 90, "high_temp should be updated value");
    }

    #[test]
    fn test_set_variable_upsert_overwrites() {
        let mut thermal = Thermal::new();
        let var_uuid = uuid!("11111111-2222-3333-4444-555555555555");

        // Set initial value
        let msg1 = thermal_req(set_variable_payload(&var_uuid, 100));
        thermal.ffa_msg_send_direct_req2(msg1).unwrap();

        // Overwrite with new value
        let msg2 = thermal_req(set_variable_payload(&var_uuid, 200));
        thermal.ffa_msg_send_direct_req2(msg2).unwrap();

        // Get should return the latest value
        let get_msg = thermal_req(get_variable_payload(&var_uuid));
        let resp = thermal.ffa_msg_send_direct_req2(get_msg).unwrap();
        assert_eq!(resp.payload().u32_at(8), 200, "data should be updated value");
    }

    #[test]
    fn test_variable_store_full_rejects() {
        let mut thermal = Thermal::new();

        // Fill all 16 slots with unique UUIDs.
        for i in 0..MAX_VARS {
            let mut uuid_bytes = [0u8; 16];
            uuid_bytes[0] = i as u8;
            let var_uuid = Builder::from_slice_le(&uuid_bytes).unwrap().into_uuid();
            let msg = thermal_req(set_variable_payload(&var_uuid, i as u32));
            let resp = thermal.ffa_msg_send_direct_req2(msg).unwrap();
            assert_eq!(resp_status(&resp), 0, "slot {i} should be accepted");
        }

        // A 17th distinct UUID has no open slot — the store rejects it.
        let mut new_uuid_bytes = [0u8; 16];
        new_uuid_bytes[0] = 0xFF;
        let new_uuid = Builder::from_slice_le(&new_uuid_bytes).unwrap().into_uuid();
        let msg = thermal_req(set_variable_payload(&new_uuid, 999));
        let resp = thermal.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_status(&resp), -1, "store is full; new UUID rejected");

        // The rejected UUID is not retrievable.
        let get_msg = thermal_req(get_variable_payload(&new_uuid));
        let resp = thermal.ffa_msg_send_direct_req2(get_msg).unwrap();
        assert_eq!(resp.payload().u64_at(0) as i64, -1, "rejected UUID absent");

        // Existing entries are untouched — every original UUID still reads back.
        for i in 0..MAX_VARS {
            let mut uuid_bytes = [0u8; 16];
            uuid_bytes[0] = i as u8;
            let var_uuid = Builder::from_slice_le(&uuid_bytes).unwrap().into_uuid();
            let get_msg = thermal_req(get_variable_payload(&var_uuid));
            let resp = thermal.ffa_msg_send_direct_req2(get_msg).unwrap();
            assert_eq!(resp.payload().u64_at(0) as i64, 0, "uuid {i} retained");
            assert_eq!(resp.payload().u32_at(8), i as u32, "uuid {i} data retained");
        }

        // Updating an existing UUID still succeeds even when the store is full.
        let mut uuid0_bytes = [0u8; 16];
        uuid0_bytes[0] = 0;
        let uuid0 = Builder::from_slice_le(&uuid0_bytes).unwrap().into_uuid();
        let msg = thermal_req(set_variable_payload(&uuid0, 4242));
        let resp = thermal.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_status(&resp), 0, "update-in-place allowed when full");
        let get_msg = thermal_req(get_variable_payload(&uuid0));
        let resp = thermal.ffa_msg_send_direct_req2(get_msg).unwrap();
        assert_eq!(resp.payload().u32_at(8), 4242, "uuid0 updated in place");
    }

    #[test]
    fn test_threshold_sensor_id_out_of_bounds() {
        let mut thermal = Thermal::new();

        // sensor_id = 8 (MAX_SENSORS) should fail
        let set_msg = thermal_req(set_threshold_payload(8, 100, 10, 50));
        let set_resp = thermal.ffa_msg_send_direct_req2(set_msg).unwrap();
        assert_eq!(resp_status(&set_resp), -1, "set with sensor_id=8 should return -1");

        // get_threshold with sensor_id=8 should also fail
        let get_msg = thermal_req(get_threshold_payload(8));
        let get_resp = thermal.ffa_msg_send_direct_req2(get_msg).unwrap();
        assert_eq!(resp_status(&get_resp), -1, "get with sensor_id=8 should return -1");

        // sensor_id = 255 should also fail
        let set_msg = thermal_req(set_threshold_payload(255, 100, 10, 50));
        let set_resp = thermal.ffa_msg_send_direct_req2(set_msg).unwrap();
        assert_eq!(resp_status(&set_resp), -1, "set with sensor_id=255 should return -1");
    }

    #[test]
    fn test_multiple_sensors_independent() {
        let mut thermal = Thermal::new();

        // Set thresholds for sensor 0 and sensor 3 with different values
        let msg0 = thermal_req(set_threshold_payload(0, 100, 10, 50));
        thermal.ffa_msg_send_direct_req2(msg0).unwrap();

        let msg3 = thermal_req(set_threshold_payload(3, 200, 20, 90));
        thermal.ffa_msg_send_direct_req2(msg3).unwrap();

        // Verify sensor 0 has its own values
        let get0 = thermal_req(get_threshold_payload(0));
        let resp0 = thermal.ffa_msg_send_direct_req2(get0).unwrap();
        assert_eq!(resp0.payload().u32_at(8), 100, "sensor 0 timeout");
        assert_eq!(resp0.payload().u32_at(12), 10, "sensor 0 low_temp");
        assert_eq!(resp0.payload().u32_at(16), 50, "sensor 0 high_temp");

        // Verify sensor 3 has its own values
        let get3 = thermal_req(get_threshold_payload(3));
        let resp3 = thermal.ffa_msg_send_direct_req2(get3).unwrap();
        assert_eq!(resp3.payload().u32_at(8), 200, "sensor 3 timeout");
        assert_eq!(resp3.payload().u32_at(12), 20, "sensor 3 low_temp");
        assert_eq!(resp3.payload().u32_at(16), 90, "sensor 3 high_temp");
    }

    #[test]
    fn test_cooling_policy_sensor_id_out_of_bounds() {
        let mut thermal = Thermal::new();

        let msg = thermal_req(set_cooling_policy_payload(8, 1));
        let resp = thermal.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(
            resp_status(&resp),
            -1,
            "cooling policy with sensor_id=8 should return -1"
        );
    }
}
