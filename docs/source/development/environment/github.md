# GitHub

This page provides a quick reference of commonly used git commands within the repository, alongside some basic terminology and concepts. It is an overview of _how_ to use git.

:::{note}
For info on what to name branches, or general GitHub standards, see <project:/standards/github.md>. It provides on overview of _what_ specifics to use within the git commands.
:::

## Interactive Tutorial

For a full interactive tutorial on git, head to [this website](https://learngitbranching.js.org/). It contains a fully interactive tutorial going through all the commands of git, and providing a safe workspace to try them out in.

The main sections to work through within the levels menu are the 'Introduction Sequence' in the 'Main' tab, and the 'Push & Pull' tutorials in the 'Remote' tab. To access the levels menu, head to the bottom-right corner of the screen and click the question mark, then hit 'levels' to open the menu.

## Terminology

- **Git** is a version control system (VCS) that is used for tracking changes in files. It is a local system that can interface with external systems (repositories)
- **GitHub** is a remote repository manager, which provides a central location for changes to be tracked between devices. It also provides additional functionality in regards to continuous integration/development (CI/CD) and the docs.
- **Repository** This is a collection of files and folders that git is tracking. For example, `perseus-v2` is the repository that contains all the files for the perseus rover.
- **Main** The main branch or _main_ is the production branch for all the perseus code and files. Files on the main branch are shared with everyone, and are protected by certain CI/CD commands to prevent any broken changes from entering production (for more details see <project:/standards/github.md>)
- **Branches** Each branch is an independent collection of commits, that tracks changes from a certain point in time. Think of branches like a tree, the main branch is the trunk (where everything stems from) and each branch is, well, a branch. Each branch can develop code independently from everything else, allowing changes, testing, and development to happen without breaking production.
- **Commits** A commit in it's simplest form is a set of changes. It consists of files that have been added, modified, or deleted. Each commit also includes a message describing what it has changed. The repository is made up of commits, which git tracks.
- **Push** When you make a commit following [The Standards](project:/standards/github.md), it is a local commit (only on your device). The same applies to any new branches created. A push is when you upload those commits and any new branch to the remote repository on GitHub.
- **Pull** When changes are made to the remote repository you can _pull_ changes to update your local repository and ensure you are up to date with the latest version of the `perseus-v2` repository.

For more details and further terminology see [the official docs](https://git-scm.com/docs/gitglossary).

:::{tip}
When using git it can be useful to run the command `git status` to see an overview of your current branch and changes. This is especially important when working with the CLI only, without a UI (ie. outside VSCode).
:::

## Branches

For changes to the perseus repo, you should create a new branch - there are a couple different ways to do this, but the main one is:

```console
git branch NEW_BRANCH_NAME
git checkout NEW_BRANCH_NAME
```

Or (combined version):

```console
git checkout -b NEW_BRANCH_NAME
```

:::{note}
Make sure to follow the [github standards](project:/standards/github.md) when creating and naming your branch.
:::

These commands create a new branch and switch to it. The new branch will be a copy of whatever branch you had currently checked out. To make a branch from a different branch than the one you are on, you can do:

```console
git branch NEW_BRANCH_NAME OLD_BRANCH_NAME
git checkout NEW_BRANCH_NAME
```

Or (combined version):

```
git checkout -b NEW_BRANCH_NAME OLD_BRANCH_NAME
```

When you've made changes, you can stage them. Staging your changes adds them to the next commit, essentially moving them from 'in-progress/development' to 'section done'. When making a commit it is important to remember that commits remain within your local repository _until_ you push the changes to GitHub. For details on pushing commits to the remote repository see below. You can stage changes by either using the vscode source control feature (Source control tab on left sidebar below Explorer tab), or using:

```console
git add path_to_file
```

You can also add everything in a folder:

```console
git add path_to_folder
```

Or (to add the current folder):

```console
git add .
```

If you do this while in the base folder (`~/perseus-v2`), it will add all changed files in the local repository.

Then you can push your commits to the remote GitHub repository. The first time you do this, you'll need to run `git push --set-upstream origin NEW_BRANCH_NAME` to add your local branch to the GitHub.
After that, you can just run `git push` whenever you want to update the remote GitHub branch from your local branch (after staging changes). Make sure that you follow the GitHub standards when making commits to ensure the repository remains professional.

:::{note}
When making a commit, be sure to follow the [github standards](project:/standards/github.md) to ensure consistency across the GitHub.
:::

## CI/CD

CI/CD (Continuous Integration, Continuous Deployment) is how we ensure that any changes won't break our system. You can check that your branch is passing the CI/CD checks on GitHub - there should be a green tick, a red cross, or an orange dot in the heading at the top of the table next to the last commit message. The orange dot means the checks are still being run. The green tick means all the checks were successful. The red cross means some checks failed. You can check which ones failed by clicking on the cross.

![CI/CD_GitHub](../../_static/github-ci-cd.png)

Have a look at <project:/systems/ci-cd.md> for more details on our CI/CD.

## Pull Requests

Once you've finished developing on your branch , you can open a Pull Request (PR) to merge your branch into the main branch. To do this, go to the [Perseus GitHub repo](https://github.com/ROAR-QUTRC/perseus-v2) and go to "Pull requests" then click "New pull request". Select your branch and click "View pull request". This will take you to the pull request page, where you can write a title and a description of your changes. Make sure your branch is not behind main when submitting the pull request, see <project:#merges> below for more detail.

:::{warning}
_Finished developing_ means your code is finalised, has been tested, and in your opinion should not require any more changes. Testing is required before a pull-request is made, you must be able to show/prove your code works before it can get merged into the main Github branch.
:::

For changes to the docs, please ensure you include a screenshot of the built changes to ensure they are formatted properly (see <project:/development/documentation-index.md>).

Once you're finished writing the description of your PR, you can select reviewers. For most files, the reviewers will be autoselected from the `.github/CODEOWNERS` file, which specifies who can approve changes to which files.
For example, this line:

```none
/software/web_ui/ @OMEN44 @UltraFishy
```

Means that a change to any files in the `/software/web_ui/` directory must be approved by the github users OMEN44 or UltraFishy.

Some files do not have a CODEOWNER. Changes to these can be approved by anyone in the ROAR team (anyone who has write access to the repo). An example of this is the .envrc file in the repo root or any files in the software/scripts/ directory. For changes to these files, you can assign any reviewer, but you should still assign one of the leads as they are the most likely to be looking at and approving PRs.

Reviewers need to approve every file change in the PR. If they have any problems with your changes, they will make comments on the PR on GitHub and you need to make the changes before they approve the PR. If they approve a file change, but you later decide to make another change to the same file, that file will need to be approved again.

## Merges

Once a PR is approved and is passing all CI/CD checks, that branch will be added to the merge queue. GitHub will automatically merge your branch into the main branch. If your branch is several commits behind main, it may not be able to be automatically merged. During development you should be regularly merging main into your branch using `git merge main`. Make sure you are on the correct branch before running this command (you should be on your branch, not main). Merging ensures that any changes someone else has made don't conflict with your changes. Make sure you run `git pull` to update your local main branch before running this though, or there won't be any changes to be merged.

:::{note}
Keep track of your PR and it's current status while waiting for it to be merged. Make sure to keep the branch up-to-date with commits from main if the PR is taking a while to be approved or rejected.
:::

## Updating your local repo

The repo on your computer won't update automatically. To do this, you need to periodically run `git pull`, which updates your current branch with the latest changes from it's corresponding counterpart on the GitHub. To update all branches you should instead run `git fetch --all`.

Updating with the latest changes from remote won't get rid of local branches that have been deleted or merged on the GitHub. To do this, you can run `git remote prune origin`, which removes any branches on your local repo that no longer exist on the GitHub. You can also combine these two methods using `git pull --prune` or `git fetch --all --prune`.
