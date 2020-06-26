# Contributions

All contributors must first sign the [Particle Individual Contributor License Agreement (CLA)](https://docs.google.com/a/spark.io/forms/d/1_2P-vRKGUFg5bmpcKLHO_qNZWGi5HKYnfrrkd-sbZoA/viewform), which is based on the Google CLA, and provides the Particle team a license to re-distribute your contributions.

Whenever possible, please follow these guidelines for contributions:

- Keep each pull request small and focused on a single feature or bug fix.
- Familiarize yourself with the code base, and follow the formatting principles adhered to in the surrounding code.
- Wherever possible, provide unit tests for your contributions.
- If the changes have an impact application developers, then those changes should be described in the documentation.


# Tests and Documentation

Any changes that affect firmware application developers, such as a new class or API, or change in existing behavior should have an accompanying PR to the `particle/docs` repo describing the changes. This ensures the documentation is kept up to date with changes to the firmware.


# Subtrees

The repository imports content from other repos via git subtrees. These are the current
subtrees:

- lib/AM1805

(you can find an up-to-date list by running git log | grep git-subtree-dir | awk '{ print $2 }')

When making commits, do not mix commits that span subtrees. E.g. a commit that posts
changes to files in lib/AM1805 and src should be avoided.
No real harm is done, but the commit messages can be confusing since those files outside the subtree
will be removed when the subtree is pushed to the upstream repo.

To avoid any issues with subtres, it's simplest to make all changes to the upstream repo via
PRs from a separate working copy, just as you would with any regular repo. Then pull these changes
into the subtree in this repo.


