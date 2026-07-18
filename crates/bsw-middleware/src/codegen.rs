//! Checked middleware model parser and deterministic Rust generator.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use crate::{PrimitiveType, MAX_MEMBER_ID};

/// Checked schema/model error with source line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SchemaError {
    /// One-based input line, or zero for whole-model checks.
    pub line: usize,
    /// Error classification.
    pub kind: SchemaErrorKind,
}

/// Configuration error classification.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SchemaErrorKind {
    /// Unknown declaration keyword or wrong field count.
    Syntax,
    /// Name is not a valid Rust-style identifier.
    InvalidName,
    /// Numeric identifier is outside its protocol range.
    InvalidId,
    /// A name or identifier is declared twice in the same namespace.
    Duplicate,
    /// A referenced type or service is not declared.
    UnknownReference,
    /// Route source equals target or duplicates another route.
    InvalidRoute,
}

/// Owned checked intermediate representation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Model {
    /// Model/module name.
    pub name: String,
    /// Named primitive aliases.
    pub types: Vec<TypeDef>,
    /// Declared services.
    pub services: Vec<ServiceDef>,
    /// Cluster routes.
    pub routes: Vec<RouteDef>,
}

/// Named primitive alias.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypeDef {
    /// Alias name.
    pub name: String,
    /// Primitive representation.
    pub primitive: PrimitiveType,
}

/// Checked service.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ServiceDef {
    /// Service name.
    pub name: String,
    /// Service identifier.
    pub id: u16,
    /// Methods in declaration order.
    pub methods: Vec<MethodDef>,
}

/// Checked service method.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MethodDef {
    /// Method name.
    pub name: String,
    /// Member identifier.
    pub id: u16,
    /// Request type name.
    pub request: String,
    /// Response type name.
    pub response: String,
}

/// Checked service route.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteDef {
    /// Service name.
    pub service: String,
    /// Source cluster.
    pub source: u8,
    /// Target cluster.
    pub target: u8,
}

/// Parse and validate the line-oriented middleware schema.
///
/// Grammar (whitespace-separated, `#` comments):
/// `model NAME`, `type NAME PRIMITIVE`, `service NAME ID`,
/// `method SERVICE NAME ID REQUEST RESPONSE`, `route SERVICE SRC DST`.
#[allow(clippy::too_many_lines)] // Line-aware validation is one deterministic parsing pass.
pub fn parse_model(input: &str) -> Result<Model, SchemaError> {
    let mut model_name: Option<(usize, String)> = None;
    let mut types = Vec::new();
    let mut raw_services: Vec<(usize, String, u16)> = Vec::new();
    let mut raw_methods: Vec<(usize, String, MethodDef)> = Vec::new();
    let mut routes = Vec::new();

    for (offset, raw) in input.lines().enumerate() {
        let line = offset + 1;
        let text = raw.split('#').next().unwrap_or_default().trim();
        if text.is_empty() {
            continue;
        }
        let fields: Vec<&str> = text.split_whitespace().collect();
        match fields.as_slice() {
            ["model", name] => {
                valid_name(name, line)?;
                if model_name.replace((line, (*name).to_owned())).is_some() {
                    return error(line, SchemaErrorKind::Duplicate);
                }
            }
            ["type", name, primitive] => {
                valid_name(name, line)?;
                let primitive = parse_primitive(primitive).ok_or(SchemaError {
                    line,
                    kind: SchemaErrorKind::UnknownReference,
                })?;
                types.push(TypeDef {
                    name: (*name).to_owned(),
                    primitive,
                });
            }
            ["service", name, id] => {
                valid_name(name, line)?;
                let id = number(id)
                    .filter(|id| *id > 0 && *id < u64::from(u16::MAX))
                    .ok_or(SchemaError {
                        line,
                        kind: SchemaErrorKind::InvalidId,
                    })? as u16;
                raw_services.push((line, (*name).to_owned(), id));
            }
            ["method", service, name, id, request, response] => {
                valid_name(service, line)?;
                valid_name(name, line)?;
                let id = number(id)
                    .filter(|id| *id > 0 && *id <= u64::from(MAX_MEMBER_ID))
                    .ok_or(SchemaError {
                        line,
                        kind: SchemaErrorKind::InvalidId,
                    })? as u16;
                raw_methods.push((
                    line,
                    (*service).to_owned(),
                    MethodDef {
                        name: (*name).to_owned(),
                        id,
                        request: (*request).to_owned(),
                        response: (*response).to_owned(),
                    },
                ));
            }
            ["route", service, source, target] => {
                valid_name(service, line)?;
                let source = number(source)
                    .filter(|id| u8::try_from(*id).is_ok())
                    .ok_or(SchemaError {
                        line,
                        kind: SchemaErrorKind::InvalidId,
                    })? as u8;
                let target = number(target)
                    .filter(|id| u8::try_from(*id).is_ok())
                    .ok_or(SchemaError {
                        line,
                        kind: SchemaErrorKind::InvalidId,
                    })? as u8;
                if source == target {
                    return error(line, SchemaErrorKind::InvalidRoute);
                }
                routes.push((
                    line,
                    RouteDef {
                        service: (*service).to_owned(),
                        source,
                        target,
                    },
                ));
            }
            _ => return error(line, SchemaErrorKind::Syntax),
        }
    }

    let (model_line, name) = model_name.ok_or(SchemaError {
        line: 0,
        kind: SchemaErrorKind::Syntax,
    })?;
    let mut type_names = BTreeSet::from([
        "Unit".to_owned(),
        "bool".to_owned(),
        "u8".to_owned(),
        "u16".to_owned(),
        "u32".to_owned(),
        "i16".to_owned(),
    ]);
    for type_def in &types {
        if !type_names.insert(type_def.name.clone()) {
            return error(model_line, SchemaErrorKind::Duplicate);
        }
    }
    let mut service_names = BTreeSet::new();
    let mut service_ids = BTreeSet::new();
    let mut services = Vec::new();
    for (line, service_name, id) in raw_services {
        if !service_names.insert(service_name.clone()) || !service_ids.insert(id) {
            return error(line, SchemaErrorKind::Duplicate);
        }
        services.push(ServiceDef {
            name: service_name,
            id,
            methods: Vec::new(),
        });
    }
    let indices: BTreeMap<String, usize> = services
        .iter()
        .enumerate()
        .map(|(index, service)| (service.name.clone(), index))
        .collect();
    for (line, service_name, method) in raw_methods {
        if !type_names.contains(&method.request) || !type_names.contains(&method.response) {
            return error(line, SchemaErrorKind::UnknownReference);
        }
        let Some(index) = indices.get(&service_name).copied() else {
            return error(line, SchemaErrorKind::UnknownReference);
        };
        if services[index]
            .methods
            .iter()
            .any(|old| old.name == method.name || old.id == method.id)
        {
            return error(line, SchemaErrorKind::Duplicate);
        }
        services[index].methods.push(method);
    }
    let mut checked_routes = Vec::new();
    let mut route_keys = BTreeSet::new();
    for (line, route) in routes {
        if !service_names.contains(&route.service) {
            return error(line, SchemaErrorKind::UnknownReference);
        }
        if !route_keys.insert((route.service.clone(), route.source, route.target)) {
            return error(line, SchemaErrorKind::InvalidRoute);
        }
        checked_routes.push(route);
    }
    Ok(Model {
        name,
        types,
        services,
        routes: checked_routes,
    })
}

/// Render deterministic, rustfmt-compatible `no_std` Rust source.
pub fn generate(model: &Model) -> String {
    let mut output = String::new();
    writeln!(
        output,
        "// @generated by bsw-middleware-codegen; DO NOT EDIT."
    )
    .unwrap();
    writeln!(output, "use ::bsw_middleware::{{ClusterRoute, MemberDescriptor, PrimitiveType, ServiceDescriptor}};\n").unwrap();
    for type_def in &model.types {
        writeln!(
            output,
            "pub type {} = {};",
            type_def.name,
            rust_type(type_def.primitive)
        )
        .unwrap();
    }
    if !model.types.is_empty() {
        output.push('\n');
    }
    for service in &model.services {
        let upper = screaming(&service.name);
        writeln!(
            output,
            "pub const {upper}_MEMBERS: &[MemberDescriptor] = &["
        )
        .unwrap();
        for method in &service.methods {
            writeln!(output, "    MemberDescriptor {{").unwrap();
            writeln!(output, "        name: \"{}\",", method.name).unwrap();
            writeln!(output, "        id: {},", method.id).unwrap();
            writeln!(
                output,
                "        request: PrimitiveType::{},",
                primitive_variant(model, &method.request)
            )
            .unwrap();
            writeln!(
                output,
                "        response: PrimitiveType::{},",
                primitive_variant(model, &method.response)
            )
            .unwrap();
            writeln!(output, "    }},").unwrap();
        }
        writeln!(
            output,
            "];\npub const {upper}: ServiceDescriptor = ServiceDescriptor {{"
        )
        .unwrap();
        writeln!(output, "    name: \"{}\",", service.name).unwrap();
        writeln!(output, "    id: {},", service.id).unwrap();
        writeln!(output, "    members: {upper}_MEMBERS,").unwrap();
        writeln!(output, "}};\n").unwrap();
    }
    if model.routes.len() == 1 {
        output.push_str("pub const ROUTES: &[ClusterRoute] = &[ClusterRoute {\n");
    } else {
        output.push_str("pub const ROUTES: &[ClusterRoute] = &[\n");
    }
    for route in &model.routes {
        let id = model
            .services
            .iter()
            .find(|service| service.name == route.service)
            .unwrap()
            .id;
        if model.routes.len() == 1 {
            writeln!(output, "    service_id: {id},").unwrap();
            writeln!(output, "    source_cluster: {},", route.source).unwrap();
            writeln!(output, "    target_cluster: {},", route.target).unwrap();
        } else {
            writeln!(output, "    ClusterRoute {{").unwrap();
            writeln!(output, "        service_id: {id},").unwrap();
            writeln!(output, "        source_cluster: {},", route.source).unwrap();
            writeln!(output, "        target_cluster: {},", route.target).unwrap();
            writeln!(output, "    }},").unwrap();
        }
    }
    output.push_str(if model.routes.len() == 1 {
        "}];\n"
    } else {
        "];\n"
    });
    output
}

fn error<T>(line: usize, kind: SchemaErrorKind) -> Result<T, SchemaError> {
    Err(SchemaError { line, kind })
}

fn valid_name(name: &str, line: usize) -> Result<(), SchemaError> {
    let mut chars = name.chars();
    if !chars
        .next()
        .is_some_and(|ch| ch == '_' || ch.is_ascii_alphabetic())
        || !chars.all(|ch| ch == '_' || ch.is_ascii_alphanumeric())
    {
        return error(line, SchemaErrorKind::InvalidName);
    }
    Ok(())
}

fn number(text: &str) -> Option<u64> {
    text.strip_prefix("0x").map_or_else(
        || text.parse().ok(),
        |hex| u64::from_str_radix(hex, 16).ok(),
    )
}

fn parse_primitive(name: &str) -> Option<PrimitiveType> {
    Some(match name {
        "Unit" => PrimitiveType::Unit,
        "bool" => PrimitiveType::Bool,
        "u8" => PrimitiveType::U8,
        "u16" => PrimitiveType::U16,
        "u32" => PrimitiveType::U32,
        "i16" => PrimitiveType::I16,
        _ => return None,
    })
}

fn rust_type(primitive: PrimitiveType) -> &'static str {
    match primitive {
        PrimitiveType::Unit => "()",
        PrimitiveType::Bool => "bool",
        PrimitiveType::U8 => "u8",
        PrimitiveType::U16 => "u16",
        PrimitiveType::U32 => "u32",
        PrimitiveType::I16 => "i16",
    }
}

fn primitive_variant(model: &Model, name: &str) -> &'static str {
    let primitive = parse_primitive(name)
        .or_else(|| {
            model
                .types
                .iter()
                .find(|item| item.name == name)
                .map(|item| item.primitive)
        })
        .unwrap();
    match primitive {
        PrimitiveType::Unit => "Unit",
        PrimitiveType::Bool => "Bool",
        PrimitiveType::U8 => "U8",
        PrimitiveType::U16 => "U16",
        PrimitiveType::U32 => "U32",
        PrimitiveType::I16 => "I16",
    }
}

fn screaming(name: &str) -> String {
    let mut result = String::new();
    for (index, ch) in name.chars().enumerate() {
        if ch.is_ascii_uppercase() && index > 0 {
            result.push('_');
        }
        result.push(ch.to_ascii_uppercase());
    }
    result
}
