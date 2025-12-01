This ESP32 project implements several tasks on core 0 and core 1 with varying task priorities.  We do this to learn about Rate Monotonic theory.  We built this
in Rust for memory safety.  You should still run cargo clippy on the code, as there are things that could be optimized.  Hope this project can help you out.
For the GPS task to work properly, you should conect your NEO GPS unit etc. to the ESP32.  I used a cheap NEO GPS unit I got from Amazon.  If you don't need GPS, I have the same project without GPS support in another one of my repos.  
