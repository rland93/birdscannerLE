//! Length-Type-Value iteration (BAP v1.0 Generic Audio LTV format, also used for
//! metadata): `Length` (1 octet, counts `Type` + `Value`), `Type` (1 octet), `Value`.

/// Iterator over LTV structures.
///
/// Yields `Ok((type, value))`, or once `Err(type)` if an entry overruns the buffer
/// (`type` is 0x00 when not even the type octet is present); iteration stops after an
/// error. Zero-length entries carry no type and are skipped.
pub(crate) struct Ltvs<'a> {
    rest: &'a [u8],
}

pub(crate) fn iter(data: &[u8]) -> Ltvs<'_> {
    Ltvs { rest: data }
}

impl<'a> Iterator for Ltvs<'a> {
    type Item = Result<(u8, &'a [u8]), u8>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let (&len, rest) = self.rest.split_first()?;
            let len = usize::from(len);
            if len == 0 {
                self.rest = rest;
                continue;
            }
            if rest.len() < len {
                self.rest = &[];
                return Some(Err(rest.first().copied().unwrap_or(0)));
            }
            let (entry, tail) = rest.split_at(len);
            self.rest = tail;
            return Some(Ok((entry[0], &entry[1..])));
        }
    }
}
