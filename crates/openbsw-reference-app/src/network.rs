//! Host-loopback UDP/TCP echo workflows used by the reference application.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream, UdpSocket};
use std::thread;

/// Run one UDP echo transaction on an ephemeral loopback port.
pub fn udp_echo_once(payload: &[u8]) -> std::io::Result<Vec<u8>> {
    let server = UdpSocket::bind(("127.0.0.1", 0))?;
    server.set_read_timeout(Some(std::time::Duration::from_secs(1)))?;
    let address = server.local_addr()?;
    let worker = thread::spawn(move || -> std::io::Result<()> {
        let mut bytes = [0u8; 512];
        let (length, peer) = server.recv_from(&mut bytes)?;
        server.send_to(&bytes[..length], peer)?;
        Ok(())
    });
    let client = UdpSocket::bind(("127.0.0.1", 0))?;
    client.set_read_timeout(Some(std::time::Duration::from_secs(1)))?;
    client.send_to(payload, address)?;
    let mut output = vec![0u8; payload.len()];
    let (length, _) = client.recv_from(&mut output)?;
    output.truncate(length);
    worker.join().expect("UDP echo worker did not panic")?;
    Ok(output)
}

/// Run one TCP echo transaction on an ephemeral loopback port.
pub fn tcp_echo_once(payload: &[u8]) -> std::io::Result<Vec<u8>> {
    let listener = TcpListener::bind(("127.0.0.1", 0))?;
    let address = listener.local_addr()?;
    let worker = thread::spawn(move || -> std::io::Result<()> {
        let (mut stream, _) = listener.accept()?;
        let mut bytes = [0u8; 512];
        let length = stream.read(&mut bytes)?;
        stream.write_all(&bytes[..length])
    });
    let mut client = TcpStream::connect(address)?;
    client.set_read_timeout(Some(std::time::Duration::from_secs(1)))?;
    client.write_all(payload)?;
    let mut output = vec![0u8; payload.len()];
    client.read_exact(&mut output)?;
    worker.join().expect("TCP echo worker did not panic")?;
    Ok(output)
}
