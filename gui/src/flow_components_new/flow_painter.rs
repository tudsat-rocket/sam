use crate::flow_components::flow_component::ConnectionState;

pub enum FlowPainter {
    Missing,
    GenericValve(ConnectionState),
}