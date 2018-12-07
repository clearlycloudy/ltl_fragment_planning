pub fn version()-> & 'static str {
    "0.0.0"
}

pub trait TransitionSys< Action, State > {
    fn new() -> Self;
    fn query_state( &self ) -> State;
    fn transition( &mut self, a: Action );
}
