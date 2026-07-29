use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClipperError {
    message: String,
}

impl ClipperError {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl fmt::Display for ClipperError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ClipperError {}

pub type Result<T> = std::result::Result<T, ClipperError>;
