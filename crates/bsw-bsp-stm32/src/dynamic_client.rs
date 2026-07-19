//! Safe fixed-capacity replacement for upstream `bspDynamicClient`.
//!
//! Channel-to-client mappings are explicit, bounded, and lifetime checked.
//! A client occupies one slot regardless of how many channels reference it;
//! clearing its last channel releases the slot for deterministic reuse.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DynamicClientError {
    ChannelOutOfRange,
    ClientChannelOutOfRange,
    ClientCapacity,
}

pub struct DynamicClientRegistry<'a, T: ?Sized, const CHANNELS: usize, const CLIENTS: usize> {
    channels: [Option<(usize, u16)>; CHANNELS],
    clients: [Option<&'a T>; CLIENTS],
}

impl<'a, T: ?Sized, const CHANNELS: usize, const CLIENTS: usize>
    DynamicClientRegistry<'a, T, CHANNELS, CLIENTS>
{
    #[must_use]
    pub const fn new() -> Self {
        Self {
            channels: [None; CHANNELS],
            clients: [None; CLIENTS],
        }
    }

    pub fn set(
        &mut self,
        channel: usize,
        client_channel: u16,
        client: &'a T,
    ) -> Result<(), DynamicClientError> {
        if channel >= CHANNELS {
            return Err(DynamicClientError::ChannelOutOfRange);
        }
        if client_channel == u16::MAX {
            return Err(DynamicClientError::ClientChannelOutOfRange);
        }
        let existing = self.clients.iter().position(|entry| {
            entry.is_some_and(|registered| core::ptr::eq::<T>(registered, client))
        });
        let slot = if let Some(slot) = existing {
            slot
        } else {
            let slot = self
                .clients
                .iter()
                .position(Option::is_none)
                .ok_or(DynamicClientError::ClientCapacity)?;
            self.clients[slot] = Some(client);
            slot
        };
        self.channels[channel] = Some((slot, client_channel));
        Ok(())
    }

    pub fn clear(&mut self, channel: usize) -> Result<(), DynamicClientError> {
        let mapping = self
            .channels
            .get_mut(channel)
            .ok_or(DynamicClientError::ChannelOutOfRange)?
            .take();
        let Some((slot, _)) = mapping else {
            return Ok(());
        };
        if !self
            .channels
            .iter()
            .flatten()
            .any(|(mapped_slot, _)| *mapped_slot == slot)
        {
            self.clients[slot] = None;
        }
        Ok(())
    }

    #[must_use]
    pub fn get(&self, channel: usize) -> Option<(&'a T, u16)> {
        let (slot, client_channel) = *self.channels.get(channel)?.as_ref()?;
        Some((self.clients.get(slot)?.as_ref().copied()?, client_channel))
    }

    #[must_use]
    pub fn registered_clients(&self) -> usize {
        self.clients.iter().flatten().count()
    }

    pub fn clear_all(&mut self) {
        self.channels.fill(None);
        self.clients.fill(None);
    }
}

impl<T: ?Sized, const CHANNELS: usize, const CLIENTS: usize> Default
    for DynamicClientRegistry<'_, T, CHANNELS, CLIENTS>
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    trait Client {
        fn id(&self) -> u8;
    }
    struct Example(u8);
    impl Client for Example {
        fn id(&self) -> u8 {
            self.0
        }
    }

    #[test]
    fn upstream_two_client_vector_maps_channels() {
        let first = Example(1);
        let second = Example(2);
        let mut registry: DynamicClientRegistry<'_, dyn Client, 3, 3> =
            DynamicClientRegistry::new();
        registry.set(0, 0, &first).unwrap();
        registry.set(1, 1, &first).unwrap();
        registry.set(2, 0, &second).unwrap();
        assert_eq!(
            registry
                .get(0)
                .map(|(client, channel)| (client.id(), channel)),
            Some((1, 0))
        );
        assert_eq!(
            registry
                .get(1)
                .map(|(client, channel)| (client.id(), channel)),
            Some((1, 1))
        );
        assert_eq!(
            registry
                .get(2)
                .map(|(client, channel)| (client.id(), channel)),
            Some((2, 0))
        );
        assert_eq!(registry.registered_clients(), 2);
    }

    #[test]
    fn clearing_last_mapping_releases_slot_and_invalid_access_is_safe() {
        let first = Example(1);
        let second = Example(2);
        let mut registry: DynamicClientRegistry<'_, dyn Client, 2, 1> =
            DynamicClientRegistry::new();
        registry.set(0, 0, &first).unwrap();
        assert_eq!(
            registry.set(1, 0, &second),
            Err(DynamicClientError::ClientCapacity)
        );
        registry.clear(0).unwrap();
        registry.set(1, 0, &second).unwrap();
        assert!(registry.get(0).is_none());
        assert!(registry.get(99).is_none());
        assert_eq!(
            registry.clear(99),
            Err(DynamicClientError::ChannelOutOfRange)
        );
    }
}
