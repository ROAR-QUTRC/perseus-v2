---
tocdepth: 4
---

# GitHub Standards

This document outlines our professional standards and best practices for contributing to the Perseus-v2 GitHub repository. The ROAR Project is a multi-year, multi-discipline endeavour and the Perseus-v2 repository represents the code for the current rover platform used at Australian and international competitions.

The Perseus-v2 repository contains all the software for the Perseus rover and includes:

- firmware (ESP32 based)
- hardware specific code
- shared libraries for communication such as CAN and networking,
- the custom web browser UI
- a complete ROS2 workspace for teleoperation and autonomous operation

As such it is imperative that the highest levels of professionalism are demonstrated in how this code is updated and the documentation supporting such changes.

All software development standards are found in the [Software Standards](project:/standards/software.md) page.

This pages addresses the incidental documentation associated with pushing new code to the common codebase. Therefore this page will describe the standards required for:

- branch creation
- commit messages
- pull requests

## Branch Naming Conventions

Branch names should be descriptive and follow a consistent pattern that clearly indicates the type of work being done. Use the following prefixes:

### Standard Prefixes

- `feat/` - New features or functionality
- `bugfix/` - Bug fixes
- `docs/` - Documentation changes
- `test/` - Adding or updating tests
- `chore/` - Maintenance tasks, dependency updates, etc.

These prefixes follow the [Conventional Branch](https://conventional-branch.github.io/) naming convention.

### Branch Naming Format

```
<prefix>/<brief-description>
```

#### Acceptable Examples of Branch names

```
feat/c1_lidar
test/simulation-unit-tests
docs/update-to-getting-started
```

#### Incorrect Examples

```
john-feature-branch        // No prefix or description
feat/stuff                 // Too vague
bugfix/bug                 // Not descriptive
random-changes             // No prefix, unclear purpose
```

## Commit Message Standards

Professional commit messages are essential for maintaining a clear project history and enabling effective collaboration. Poor commit messages like "added stuff" make it impossible to understand the purpose and scope of changes.

It is possible to run 'git log --oneline' at the terminal to see a summary of the latest commits. For example:

```
0290680 docs(simulation): add tutorial on launching the simulation
4c6fd68 Merge pull request #217 from ROAR-QUTRC/feat/livox-fixing
b970635 chore(perseus_sensors): format and lint
ca97a15 fix(perseus_sensors): update Livox driver to run on /tmp
d2cb256 chore(perseus_sensors): format and lint
5136806 fix(perseus_sensors): resolve Livox launch issues
3fad5bd Merge pull request #216 from ROAR-QUTRC/feat/web-ui
74913ae fix(docs): correct typo
2fe1d6d Merge pull request #215 from ROAR-QUTRC/clean/nuking
f99acf2 refactor(perseus_sensors): remove old camera code
e69a4db chore(ros_ws): format and lint
93e47cf docs(setup): update power on steps and add zellij notes
10c3757 chore(ros_ws): clean up workspace
ce2eed4 chore(ros_ws): format and lint
670896f docs: fix typos and add new documentation
```

By keeping the first line of the commit message under 72 characters it keeps this summary functional and practical to view.

### Commit Message Structure

We follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) specification, which provides a lightweight convention for creating an explicit commit history. This format makes it easier to write automated tools on top of the commit history and enables automatic version bumping and changelog generation.

```
<type>(<scope>): <description>

[optional body]

```

For full details on the specification, please refer to the [Conventional Commits website](https://www.conventionalcommits.org/).

### Commit Types

Use these standard commit types as defined by [Conventional Commits](https://www.conventionalcommits.org/):

- `feat` - New features or functionality
- `fix` - Bug fixes
- `docs` - Documentation changes
- `style` - Code style changes (formatting, semicolons, etc.)
- `refactor` - Code refactoring without changing functionality
- `perf` - Performance improvements
- `test` - Adding or updating tests
- `build` - Changes to build system or dependencies
- `ci` - CI/CD configuration changes
- `chore` - Maintenance tasks

### Commit Scopes

The scope should indicate the part of the codebase being changed:

```
- perseus_lite
- perseus_sensors
- perseus_nav
- perseus_control
- hardware
- firmware
- web-ui
- docs
```

### Writing Effective Commit Messages

#### The Subject Line

1. **Keep it concise**: Aim for 50 characters or fewer (max 72)
2. **No full stop at the end**
3. **Be specific and descriptive**

#### Acceptable Commit Message Examples

Please note the use of a line to separate the initial short summary from the body

```
feat(perseus_sensors): add M2M2 lidar support

- corrected the number of points published to /scan
- allowed param to select applicable USB device
- updated documentation with correct launch file
```

Other examples:

```
fix(perseus_control): resolve motor timeout issue
docs(perseus_nav): update obstacle avoidance documentation
refactor(web-ui): simplify dashboard component structure
test(perseus_lite): add unit tests for CAN communication
```

#### Examples of Poor Commit Messages to Avoid

```
added stuff                    // Completely uninformative
late night coding              // Unprofessional and uninformative
fix                            // Missing type, scope, and description
update                         // Missing type, scope, and description
minor changes                  // Not following format, uninformative
oops                           // Unprofessional
WIP                            // Work in progress - should not be in main history
asdf                           // Random characters
quick fix                      // Not following format, uninformative
perseus_sensors: updated       // Missing type, vague description
Fixed bug                      // Missing type and scope, not descriptive
```

### The Commit Body (Optional)

Use the commit body to add relevant detail if someone needed to learn more. Best practice is to explain:

- **What** the change does
- **Why** the change was necessary
- **How** it addresses the issue

#### Guidelines for Commit Body

- Wrap lines at 72 characters
- Separate the subject from body with a blank line
- Use bullet points for multiple changes
- Reference issues and pull requests if relevant

#### Example with Body

```
feat(web-ui): add user authentication system

Implemented JWT-based authentication with refresh token support.
This change enables secure user sessions and API access control.

- Added login/logout endpoints
- Implemented JWT token generation and validation
- Added refresh token mechanism for extended sessions
- Updated middleware to check authentication on protected routes

Relates to #123
```

## Pull Request Standards

### Pull Request Titles

Follow the same conventions as commit messages:

```
<type>(<scope>): <description>
```

Examples:

```
feat(perseus_nav): add autonomous navigation mode
fix(firmware): resolve CAN bus communication issues
docs(setup): improve installation instructions
```

### Pull Request Description

Include:

1. **Summary** of changes
2. **Motivation** for the changes
3. **Testing** approach and results
4. **Breaking changes** (if any)

#### Example PR Description Template

```markdown
## Summary

Brief description of what this PR does in 1-2 sentences.

## What Changed

- List of specific changes made
- Be concise but comprehensive
- Include any architectural decisions

## Why

Explain the motivation for these changes:

- What problem does it solve?
- What feature does it add?
- Why is this improvement needed?

## Testing

- [ ] Unit tests pass
- [ ] Integration tests pass
- [ ] Manual testing completed
- [ ] Documentation updated

### Test Instructions

1. Step-by-step instructions for testing
2. Expected behavior
3. Edge cases considered

## Breaking Changes

- List any breaking changes (or "None" if not applicable)
- Include migration instructions if needed

## Screenshots/Videos

(If applicable, add screenshots or videos to demonstrate the changes)

## Related Issues

Closes #issue_number
```

## Code Review Standards

### For Authors

1. **Self-review** your code before requesting review
2. **Write descriptive** PR titles and descriptions
3. **Keep PRs focused** - one feature/fix per PR
4. **Respond promptly** to review feedback
5. **Test thoroughly** before submitting

### For Reviewers

1. **Be constructive** and respectful in feedback
2. **Focus on the Software standards**, not personal preferences
3. **Suggest specific improvements** rather than just pointing out problems
4. **Approve** when code meets standards, even if you might do it differently
5. **Test the changes** when possible

## Workflow Best Practices

### Before Starting Work

1. Ensure your local `main` is up to date (git checkout main | git pull)
2. Create a new branch from the latest `main` (git checkout -b feat/example)
3. Use appropriate branch naming conventions

### During Development

1. Make **atomic commits** - each commit should represent a logical unit of work
2. Write meaningful commit messages for each commit
3. Push commits regularly to backup your work
4. Rebase or merge the latest `main` regularly to avoid conflicts

### Before Submitting PR

1. **Squash** related commits if possible if they represent incomplete work
2. **Rebase** onto the latest `main` branch
3. **Run all tests** and ensure they pass
4. **Update documentation** as necessary
5. **Self-review** the entire PR

### Example Workflow

```console
# Start new feature
git checkout main
git pull origin main
git checkout -b feat/user-dashboard

# Make changes and commit
git add .
git commit -m "feat(web-ui): add basic user dashboard layout"

# Continue development
git add .
git commit -m "feat(web-ui): implement dashboard data fetching"

# Update from main before PR
git checkout main
git pull origin main
git checkout feat/user-dashboard
git rebase main

# Push and create PR
git push origin feat/user-dashboard
```

## Common Mistakes to Avoid

### Branch Management

- Don't try to work directly on `main` branch
- Don't use vague branch names like `fix` or `update`
- Don't create branches for every small change, use commits

### Commit Messages

- Don't use commit messages like "fix", "update", "change"
- Don't include personal comments or timestamps
- Don't commit incomplete work to shared branches
- Don't use profanity or inappropriate language

### Pull Requests

- Don't create massive PRs with unrelated changes
- Don't submit PRs without testing
- Push commits frequently and review any CI/CD failures.

## Integration with Development Standards

By following these GitHub standards, you ensure that the project maintains a professional, organised and collaborative development environment that supports long-term maintainability and team productivity.

## Additional Resources

- [Conventional Commits Specification](https://www.conventionalcommits.org/) - Full specification and examples
- [Conventional Branch Naming](https://conventional-branch.github.io/) - Branch naming conventions
- [GitHub Issue Templates](https://docs.github.com/en/communities/using-templates-to-encourage-useful-issues-and-pull-requests/configuring-issue-templates-for-your-repository) - Setting up issue templates
