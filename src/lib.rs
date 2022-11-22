use std::marker::PhantomData;

use tokio_util::codec::{Decoder, Encoder};
use crc_any::CRCu16;
use bytes::{Buf, BufMut};
use mavlink::{
    error::{MessageReadError, MessageWriteError},
    Message,
    MavlinkVersion,
    MavHeader,
};

struct MavMessageSerde<M>(PhantomData<fn() -> M>);

#[derive(Clone, Debug)]
pub struct MavMsgWithHeader<M: mavlink::Message> {
    pub mav_version: MavlinkVersion,
    pub header:      MavHeader,
    pub msg:         M,
}

#[derive(Debug)]
pub enum MaybeRawMsg<M> {
    Standard(M),
    // TODO: Improve this.
    Raw(MavMsgRaw),
}

#[derive(Debug)]
pub struct MavMsgRaw {
    pub id:      u32,
    pub len:     u8, // Don't have type info to indicate the size
    pub payload: [u8; 48],
}

impl MavMsgRaw {
    pub fn from_template<M: mavlink::Message>(template: M) -> Self {
        let bytes = template.ser();
        assert!(bytes.len() <= 48);
        let mut payload = [0_u8; 48];
        payload[..bytes.len()].copy_from_slice(&bytes);
        Self {
            id: template.message_id(),
            len: bytes.len() as u8,
            payload,
        }
    }

    pub fn payload(&mut self) -> &mut [u8] {
        &mut self.payload[..self.len as usize]
    }
}

impl<M: mavlink::Message> mavlink::Message for MaybeRawMsg<M> {
    fn message_id(&self) -> u32 {
        match self {
            Self::Standard(msg) => msg.message_id(),
            Self::Raw(msg) => msg.id,
        }
    }

    fn message_name(&self) -> &'static str {
        match self {
            Self::Standard(msg) => msg.message_name(),
            Self::Raw(_) => "unknown",
        }
    }

    fn ser(&self) -> Vec<u8> {
        match self {
            Self::Standard(msg) => msg.ser(),
            Self::Raw(raw) => Vec::from(&raw.payload[..raw.len as usize]),
        }
    }

    fn parse(
        version: mavlink::MavlinkVersion,
        msgid: u32,
        payload: &[u8],
    ) -> Result<Self, mavlink::error::ParserError> {
        Ok(M::parse(version, msgid, payload).map_or_else(
            |_| {
                let mut p = [0_u8; 48];
                p.copy_from_slice(payload);
                MaybeRawMsg::Raw(MavMsgRaw {
                    id:      msgid,
                    len:     payload.len() as u8,
                    payload: p,
                })
            },
            MaybeRawMsg::Standard,
        ))
    }

    fn message_id_from_name(name: &str) -> Result<u32, &'static str> {
        M::message_id_from_name(name)
    }

    fn default_message_from_id(id: u32) -> Result<Self, &'static str> {
        Ok(M::default_message_from_id(id).map_or_else(
            |_| {
                Self::Raw(MavMsgRaw {
                    id,
                    len: 0,
                    payload: [0; 48],
                })
            },
            Self::Standard,
        ))
    }

    fn extra_crc(id: u32) -> u8 {
        M::extra_crc(id)
    }
}

// MAVLink v1 protocol message layout:
//
//     0 ... 1 ... 2 ... 3 ... 4 .... 5 ... 6 ...... n+6 ...... n+8
//     +-----+-----+-----+-----+------+-----+---------+----------+
//     | STX | LEN | SEQ | SYS | COMP | MSG | PAYLOAD | CHECKSUM |
//     |     | (n) |     | ID  |  ID  | ID  |         |          |
//     +-----+-----+-----+-----+------+-----+---------+----------+
//
//
// MAVLink v2 protocol message layout:
//
//     0 ... 1 ... 2 ..... 3 ..... 4 ... 5 ... 6 .... 7 .. 10 ...... n+10 ..... n+12 ....... n+25
//     +-----+-----+-------+-------+-----+-----+------+-----+---------+----------+------------+
//     | STX | LEN |  INC  |  CMP  | SEQ | SYS | COMP | MSG | PAYLOAD | CHECKSUM | SIGNATURE  |
//     |     | (n) | FLAGS | FLAGS |     | ID  |  ID  | ID  |         |          | (optional) |
//     +-----+-----+-------+-------+-----+-----+------+-----+---------+----------+------------+
//
//     STX:         1 byte
//     LEN:         1 byte
//     INC FLAGS:   1 byte
//     CMP FLAGS:   1 byte
//     SEQ:         1 byte
//     SYS ID:      1 byte
//     COMP ID:     1 byte
//     MSG ID:      3 bytes
//     PAYLOAD: 0-255 bytes
//     CHECKSUM:    2 bytes
//     SIGNATURE:  13 bytes
//

// source: https://github.com/iridia-ulb/supervisor/blob/34220c6b9627f5e6cc02a190ac3dc9bd849ab6b5/src/robot/drone/codec.rs
impl<M: Message> Decoder for MavMessageSerde<M> {
    type Item = MavMsgWithHeader<M>;
    type Error = MessageReadError;

    fn decode(&mut self, src: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        use mavlink::{MAV_STX, MAV_STX_V2};

        let index = src
            .iter()
            .position(|&b| [MAV_STX_V2, MAV_STX].contains(&b))
            .unwrap_or(src.remaining());

        // discard everything up to but excluding STX
        src.advance(index);

        let Some(&stx) = src.get(0) else { return Ok(None); };
        src.advance(1); // Skip over STX
        let Some(payload_len) = src.get(0).map(|&v| v as usize) else { return Ok(None); };

        // Must start calculating the CRC from now
        let mut crc_calc = CRCu16::crc16mcrf4cc();
        let (mut header, tail_len, has_signature, ver) = match stx {
            // MAVLink v1
            MAV_STX => {
                let header_end = 6 - 1;
                if src.remaining() < header_end {
                    return Ok(None);
                }
                let mut header = src.split_to(header_end);
                crc_calc.digest(header.as_ref());
                header.advance(1); // Skip payload len
                let tail_len = payload_len + 2; // payload + checksum
                (header, tail_len, false, MavlinkVersion::V1)
            }
            // MAVLink v2
            MAV_STX_V2 => {
                let header_end = 10 - 1;
                if src.remaining() < header_end {
                    return Ok(None);
                }
                let mut header = src.split_to(header_end);
                crc_calc.digest(header.as_ref());
                header.advance(1); // Skip payload len
                let has_signature = header.get_u8() & 1 == 1;
                header.advance(1); // Skip CMP FLAGS
                // Payload + checksum + maybe signature
                let tail_len = payload_len + 2 + if has_signature { 13 } else { 0 };
                (header, tail_len, has_signature, MavlinkVersion::V2)
            }
            // Something is not well formed
            _ => return Ok(None),
        };

        // Need more data
        if src.remaining() < tail_len {
            return Ok(None);
        }

        let mut payload = src.split_to(payload_len);
        crc_calc.digest(payload.as_ref());
        let crc = src.get_u16_le();
        if has_signature {
            // We're just going to ignore the signature for now.
            // TODO: Use this feature
            src.advance(13);
        }
        let seq = header.get_u8();
        let sysid = header.get_u8();
        let compid = header.get_u8();

        let msgid = match ver {
            MavlinkVersion::V1 => header.get_u8() as u32,
            MavlinkVersion::V2 => {
                // Header has 3 bytes now, but we need 4 to make up a u32.
                let mut bytes = [0; 4];
                header.copy_to_slice(&mut bytes[..3]);
                u32::from_le_bytes(bytes)
            }
        };
        crc_calc.digest(&[M::extra_crc(msgid)]);

        if crc_calc.get_crc() != crc {
            // CRC check failed, skip this message
            /* hack: we should have a CRC error here */
            return Ok(None);
        }

        let msg = M::parse(ver, msgid, payload.as_ref())?;
        payload.advance(payload.remaining());
        Ok(Some(MavMsgWithHeader {
            mav_version: ver,
            header: MavHeader {
                sequence:     seq,
                system_id:    sysid,
                component_id: compid,
            },
            msg,
        }))
    }
}

impl<M: mavlink::Message> Encoder<MavMsgWithHeader<MaybeRawMsg<M>>> for MavMessageSerde<M> {
    type Error = MessageWriteError;

    fn encode(
        &mut self,
        item: MavMsgWithHeader<MaybeRawMsg<M>>,
        dst: &mut bytes::BytesMut,
    ) -> Result<(), Self::Error> {
        match item.mav_version {
            MavlinkVersion::V1 => {
                mavlink::write_v1_msg(&mut dst.writer(), item.header, &item.msg)?;
            }

            MavlinkVersion::V2 => {
                mavlink::write_v2_msg(&mut dst.writer(), item.header, &item.msg)?;
            }
        }
        Ok(())
    }
}
