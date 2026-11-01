# WHIP's controllers in other languages

Python and the Windows exe are WHIP's main controllers. These are the same small controller in five languages, all reading `robot.json`, which holds WHIP's commands, buttons and sensors. Change it there and they all change.

Hold W / S to walk forward and back, A / D to turn, Q / E to crab (gaits firmware). Space or STOP stops it in any mode. These have no keys, click them in the window: OBSTACLE, WEB, FOLLOW, STAY CLOSE, HELLO, DANCE, REST. Pass the address as the first argument, or type it in.

| Language | UI | Run |
|---|---|---|
| C# | WinForms | `dotnet run --project csharp` |
| F# | WinForms | `dotnet run --project fsharp` |
| Rust | egui | `cargo run --release --manifest-path rust/Cargo.toml` |
| Go | Ebitengine | `cd go && go run .` (Go builds its own copy of `robot.json` in, so change both) |
| Julia | GLMakie | `julia julia/controller.jl` |
