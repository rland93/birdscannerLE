//! Little-endian field readers and writers shared by the server and client sides.

use crate::client::ParseError;

/// The single octet of a one-octet value.
pub(crate) fn one(v: &[u8]) -> Option<&u8> {
    match v {
        [b] => Some(b),
        _ => None,
    }
}

pub(crate) fn le16(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

pub(crate) fn le24(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], 0])
}

/// Sequential writer that panics (slice index) when the buffer is too small.
pub(crate) struct Writer<'a> {
    pub(crate) buf: &'a mut [u8],
    pub(crate) pos: usize,
}

impl<'a> Writer<'a> {
    pub(crate) fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    pub(crate) fn u8(&mut self, v: u8) {
        self.buf[self.pos] = v;
        self.pos += 1;
    }

    pub(crate) fn bytes(&mut self, v: &[u8]) {
        self.buf[self.pos..self.pos + v.len()].copy_from_slice(v);
        self.pos += v.len();
    }

    pub(crate) fn le16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }

    pub(crate) fn le24(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes()[..3]);
    }

    pub(crate) fn le32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
}

/// Sequential reader; every read fails with [`ParseError::Truncated`] past the end.
pub(crate) struct Reader<'a> {
    rest: &'a [u8],
}

impl<'a> Reader<'a> {
    pub(crate) fn new(data: &'a [u8]) -> Self {
        Self { rest: data }
    }

    pub(crate) fn bytes(&mut self, n: usize) -> Result<&'a [u8], ParseError> {
        if self.rest.len() < n {
            return Err(ParseError::Truncated);
        }
        let (head, tail) = self.rest.split_at(n);
        self.rest = tail;
        Ok(head)
    }

    pub(crate) fn u8(&mut self) -> Result<u8, ParseError> {
        Ok(self.bytes(1)?[0])
    }

    pub(crate) fn le16(&mut self) -> Result<u16, ParseError> {
        Ok(le16(self.bytes(2)?))
    }

    pub(crate) fn le24(&mut self) -> Result<u32, ParseError> {
        Ok(le24(self.bytes(3)?))
    }

    pub(crate) fn array<const N: usize>(&mut self) -> Result<[u8; N], ParseError> {
        let mut out = [0; N];
        out.copy_from_slice(self.bytes(N)?);
        Ok(out)
    }

    /// Length-prefixed (one octet) block.
    pub(crate) fn block(&mut self) -> Result<&'a [u8], ParseError> {
        let len = self.u8()?;
        self.bytes(usize::from(len))
    }

    /// Succeeds if everything was consumed.
    pub(crate) fn finish(self) -> Result<(), ParseError> {
        if self.rest.is_empty() {
            Ok(())
        } else {
            Err(ParseError::TrailingData)
        }
    }
}
