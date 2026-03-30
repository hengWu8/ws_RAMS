# ws_RAMS

This workspace contains the existing ABB IRB6700 workcell description, MoveIt configuration, and `click_to_move` demo path.

## New additive package

`src/abb_pi0_bridge` is a Phase-1 bridge package for future pi0/openpi integration.

- It does not replace the current ABB stack.
- It does not modify `click_to_move`.
- It defaults to mock policy + dry-run command output.
- It is intended as the first low-level streaming-compatible policy bridge layer.
