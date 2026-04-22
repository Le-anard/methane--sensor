// build.rs — required linker hook for esp-idf-sys
fn main() {
    embuild::espidf::sysenv::output();
}