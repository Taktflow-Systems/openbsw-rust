//! Automated getting-started workflows and deterministic oracle records.

use bsw_doip::{DiagnosticPayload, Packet, Payload, ProtocolVersion};
use bsw_time::Instant;

use crate::diagnostics::{doip_request, virtual_can_request};
use crate::{AppConfig, ReferenceApp};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WorkflowResult {
    pub name: &'static str,
    pub passed: bool,
}

pub fn run_getting_started_workflows() -> Vec<WorkflowResult> {
    let config = AppConfig::default();
    let mut app = ReferenceApp::new(config.clone()).expect("default config is valid");
    let mut results = vec![
        WorkflowResult {
            name: "startup",
            passed: app.start(Instant::from_nanos(0)).is_ok() && app.level() == 9,
        },
        WorkflowResult {
            name: "console-help",
            passed: app
                .command("help", Instant::from_nanos(1))
                .contains("can info"),
        },
        WorkflowResult {
            name: "lifecycle-level",
            passed: app.command("lc level 5", Instant::from_nanos(2)) == "level 5",
        },
        WorkflowResult {
            name: "logging-level",
            passed: app.command("logger level info", Instant::from_nanos(3)) == "logger INFO",
        },
        WorkflowResult {
            name: "simulated-io",
            passed: app
                .command("io adc 2048", Instant::from_nanos(4))
                .contains("pwm=500"),
        },
        WorkflowResult {
            name: "udp-echo",
            passed: crate::network::udp_echo_once(b"echo").is_ok_and(|bytes| bytes == b"echo"),
        },
        WorkflowResult {
            name: "tcp-echo",
            passed: crate::network::tcp_echo_once(b"echo").is_ok_and(|bytes| bytes == b"echo"),
        },
    ];
    let can = virtual_can_request(
        app.diagnostics_mut(),
        config.diagnostics,
        &[0x22, 0xcf, 0x01],
        Instant::from_nanos(5),
    );
    results.push(WorkflowResult {
        name: "uds-docan-multiframe",
        passed: can.is_ok_and(|response| response.bytes().len() == 27),
    });
    let request = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::DiagnosticMessage(DiagnosticPayload {
            source_address: config.diagnostics.tester_address,
            target_address: config.diagnostics.entity_address,
            data: &[0x3e, 0x00],
        }),
    };
    let mut input = [0u8; 64];
    let mut output = [0u8; 64];
    let doip_ok = request
        .encode(&mut input)
        .ok()
        .and_then(|length| {
            doip_request(
                app.diagnostics_mut(),
                &input[..length],
                Instant::from_nanos(6),
                &mut output,
            )
            .ok()
        })
        .and_then(|length| Packet::parse(&output[..length]).ok())
        .is_some_and(|packet| {
            matches!(
                packet.payload,
                Payload::DiagnosticMessage(DiagnosticPayload {
                    data: [0x7e, 0x00],
                    ..
                })
            )
        });
    results.push(WorkflowResult {
        name: "uds-doip-shared-state",
        passed: doip_ok,
    });
    results.push(WorkflowResult {
        name: "shutdown",
        passed: app.shutdown(Instant::from_nanos(7)).is_ok() && app.level() == 0,
    });
    results
}

pub fn oracle_json(source: &str) -> String {
    format!(
        concat!(
            "{{\n  \"protocol_version\": 1,\n",
            "  \"scenario\": \"reference-app-host\",\n",
            "  \"upstream_commit\": \"ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77\",\n",
            "  \"records\": [\n",
            "    {{\"sequence\":0,\"kind\":\"lifecycle\",\"source\":\"{}\",\"input\":\"startup\",\"output\":\"level-9\",\"state\":\"running\"}},\n",
            "    {{\"sequence\":1,\"kind\":\"console\",\"source\":\"{}\",\"input\":\"help\",\"output\":\"command-tree\",\"state\":\"running\"}},\n",
            "    {{\"sequence\":2,\"kind\":\"can\",\"source\":\"{}\",\"input\":\"124:0102\",\"output\":\"125:0102\",\"state\":\"running\"}},\n",
            "    {{\"sequence\":3,\"kind\":\"diagnostic\",\"source\":\"{}\",\"input\":\"docan:22cf01\",\"output\":\"62cf01:24-bytes\",\"state\":\"default\"}},\n",
            "    {{\"sequence\":4,\"kind\":\"diagnostic\",\"source\":\"{}\",\"input\":\"doip:3e00\",\"output\":\"7e00\",\"state\":\"default\"}},\n",
            "    {{\"sequence\":5,\"kind\":\"storage\",\"source\":\"{}\",\"input\":\"restart\",\"output\":\"value-restored\",\"state\":\"running\"}},\n",
            "    {{\"sequence\":6,\"kind\":\"lifecycle\",\"source\":\"{}\",\"input\":\"shutdown\",\"output\":\"level-0\",\"state\":\"stopped\"}}\n",
            "  ]\n}}\n"
        ),
        source, source, source, source, source, source, source
    )
}
