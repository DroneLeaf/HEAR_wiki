# HEAR-CLI for Software Stack Git Management

Managing the software stack for HEAR projects often involves extensive manual work with Git. Tasks like cloning multiple repositories, managing branches, synchronizing changes, and ensuring consistency across the stack can be time-consuming and error-prone.

The HEAR-CLI software-stack related programs are designed to automate these repetitive tasks.

## What is repository scoping?
The file `scripts/functions/software_stack/scoped_repos.sh` contains the repositories that are scoped in the software-stack related hear-cli programs. This is because we do not want all submodules to be included in branch creation/modification/merger, e.g. PX4-Autopilot vast submodules.


## Common use cases of the hear-cli software-stack programs

### Case I: I want to develop a new feature in branch `feature/backflip-fast`
Use `software_stack_create_branch` to create `feature/backflip-fast` from `dev-sitl`.

### Case II: I have applied hot-fixes to `dev` and want them merged back to `dev-sitl`
Run `software_stack_merge_branches_to_newest`. Verify both `dev` and `dev-sitl` are the same by running `software_stack_compare_branches`.

### Case III: I have updated `mavlink` repo in `HEAR_Util` at branch `feature/backflip-fast`. I want to update all submodule references and update `LeafMC` reference to the new `mavlink` commit.

Use `software_stack_commit_submodules_to_branch` and use branch `feature/backflip-fast` as argument.

### Case IV: I want to merge branch `feature/backflip-fast-a` into `feature/backflip-fast`.
Use `software_stack_merge_branches` and follow the interactive parameters.

### Case V: I want to delete branch `feature/backflip-fast-b` altogether
Use `software_stack_delete_branch`
**Note:** primary branches remotes, i.e. `main,dev,dev-sitl`, are protected against deletion