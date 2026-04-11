# Third-Party Patch Snapshots

This directory captures local modifications made inside nested upstream git dependencies that are not pushed as part of the top-level `ws_RAMS` repository history.

Current patch snapshots:

- `abb_ros2_upstream.patch`
- `abb_librws.patch`
- `abb_egm_rws_managers.patch`

Why these files exist:

- `ws_RAMS` tracks these dependencies as gitlinks.
- Their local working trees currently contain important changes related to the first working RobotStudio + pi0 integration.
- Pushing only the top-level `ws_RAMS` repo would otherwise lose the exact nested diffs.

These patch files are meant to preserve the current engineering state in GitHub until the nested repositories are synced in a cleaner way.
