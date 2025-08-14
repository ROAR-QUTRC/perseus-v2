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

All software develop standards are found in the Software Standards page.

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

### Branch Naming Format

```
<prefix>/<brief-description>
```

#### Acceptable Examples of Branch names

```
- feat/c1_lidar
- test/simulation-unit-tests
- docs/update-to-getting-started
```

#### Incorrect Examples

```
john-feature-branch        // No prefix or description
feat/stuff                 // Too vague
bugfix/bug                   // Not descriptive
random-changes            // No prefix, unclear purpose
```

## Commit Message Standards

Professional commit messages are essential for maintaining a clear project history and enabling effective collaboration. Poor commit messages like "added stuff" make it impossible to understand the purpose and scope of changes.

It is possible to run 'git log --oneline' at the terminal to see a summary of the latest commits. For example:

```
0290680 docs: Added a tutorial on how to launch the simulation
4c6fd68 Merge pull request #217 from ROAR-QUTRC/feat/livox-fixing
b970635 (origin/feat/livox-fixing, feat/livox-fixing) chore: Format and lint
ca97a15 Updated Livox driver to run on /tmp
d2cb256 chore: Format and lint
5136806 Fixed Livox Launch
3fad5bd (origin/i2c-node) Merge pull request #216 from ROAR-QUTRC/feat/web-ui
74913ae fix: typo
2fe1d6d Merge pull request #215 from ROAR-QUTRC/clean/nuking
f99acf2 Removed old camera code
e69a4db chore: Format and lint
93e47cf Updated power on steps and added note about zellij use
10c3757 Cleaning ros_ws
ce2eed4 chore: Format and lint
670896f Fixed typos and added new docs
```

By keeping the first line of the commit message under 72 characters it keeps this summary functional and practical to view.

### Commit Message Structure

Follow the conventional commit format:

```
<type>[package]: <description>

[optional body]

```

### Commit Packages Description

As a convention in this project, each commit message should start with the relevant part of the codebase being amended

Examples include

```
- perseus_lite: xxxxxxxx
- docs: xxxxxxxx
- hardware: xxxxxxxx
- perseus_sensors: xxxxxxx
```

### Writing Effective Commit Messages

#### The Subject Line

2. **Keep it concise**: Aim for 50 characters or fewer (max 72)
3. **Capitalize the first letter**
4. **No full stop at the end**
5. **Be specific and descriptive**

//TODO
add a visual of git log --oneline

#### Acceptable Examples

Add whitespace

//TODO

#### Examples of Poor Commit Messages to Avoid

```
added stuff                    // Completely uninformative
late night coding              // Unprofessional and uninformative
fix                           // What was fixed?
update                        // What was updated?
minor changes                 // What changes?
oops                         // Unprofessional
WIP                          // Work in progress - should not be in main history
asdf                         // Random characters
quick fix                    // What was fixed?
```

### The Commit Body (Optional)

Use the commit body to add relevant detail if someone needed to learn more. Explain:

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
//TODO

Closes #123
```

## Pull Request Standards

### Pull Request Titles

Follow the same conventions as commit messages:

```
<type>[optional scope]: <description>
```

### Pull Request Description

Include:

1. **Summary** of changes
2. **Motivation** for the changes
3. **Testing** approach and results
4. **Breaking changes** (if any)

#### Example PR Description Template

//TODO

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

3. Ensure your local `main` is up to date (git checkout main | git pull)
1. Create a new branch from the latest `main` (git checkout -b feat/example)
1. Use appropriate branch naming conventions

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

```bash
# Start new feature
git checkout main
git pull origin main
git checkout -b feat/user-dashboard

# Make changes and commit
git add .
git commit -m "feat: add basic user dashboard layout"

# Continue development
git add .
git commit -m "feat: implement dashboard data fetching"

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
- Push commits frequently and review any CI/CD failures

## Integration with Development Standards

By following these GitHub standards, you ensure that the project maintains a professional, organised and collaborative development environment that supports long-term maintainability and team productivity.
