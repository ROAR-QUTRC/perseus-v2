# GitHub

For info on what to name branches, or general GitHub standards, see <project:/standards/github.md>

## Branches

For changes to the perseus repo, you should create a new branch - there are a couple different ways to do this, but the main one is:

```console
git branch NEW_BRANCH_NAME
git checkout NEW_BRANCH_NAME
```

Or (combined version):

```
git checkout -b NEW_BRANCH_NAME
```

These commands create a new branch and switch to it. The new branch will be a copy of whatever branch you had currently checked out. To make a branch from a different branch than the one you are on, you can do:

```console
git branch NEW_BRANCH_NAME OLD_BRANCH_NAME
git checkout NEW_BRANCH_NAME
```

Or (combined version):

```
git checkout -b NEW_BRANCH_NAME OLD_BRANCH_NAME
```

When you've made changes, you can stage them - either using the vscode source control feature, or using:

```console
git add path_to_file
git commit -m "COMMIT MESSAGE"
```

Then you can push your commits to GitHub. The first time you do this, you'll need to run `git push --set-upstream origin NEW_BRANCH_NAME` to get your local branch on the GitHub.
Then after that, you can just run `git push` whenever you want to update the GitHub branch from your local branch (after staging changes).

## CI/CD

CI/CD (Continuous Integration, Continuous Deployment) is how we ensure that any changes won't break our system. You can check that your branch is passing the CI/CD checks on GitHub - there should be a green tick, a red cross, or an orange dot in the heading at the top of the table next to the last commit message. The orange dot means the checks are still being run. The green tick means all the checks were successful. The red cross means some checks failed. You can check which ones failed by clicking on the cross.

![CI/CD_GitHub](../../_static/github-ci-cd.png)

Checkout <project:/systems/ci-cd.md> for more details on our CI/CD.

## Pull Requests

Once you've finished developing on your branch, you can open a Pull Request (PR) to merge your branch into the main branch. To do this, go to the [Perseus GitHub repo](https://github.com/ROAR-QUTRC/perseus-v2) and go to "Pull requests" then click "New pull request". Select your branch and click "View pull request". This will take you to the pull request page, where you can write a title and a description of your changes.

For changes to the docs, please ensure you include a screenshot of the built changes to ensure they are formatted properly (see <project:/development/documentation-index.md>).

Once you're finished writing the description of your PR, you can select reviewers. For most files, the reviewers will be autoselected from the `.github/CODEOWNERS` file, which specifies who can approve changes to which files.
For example, this line:

```
/software/web_ui/ @OMEN44 @UltraFishy
```

Means that a change to any files in the `/software/web_ui/` directory must be approved by the github users OMEN44 or UltraFishy.

Some files do not have a CODEOWNER. Changes to these can be approved by anyone in the ROAR team (anyone who has write access to the repo). An example of this is the .envrc file in the repo root or any files in the software/scripts/ directory. For changes to these files, you can assign any reviewer, but you should still assign one of the leads as they are the most likely to be looking at and approving PRs.

Reviewers need to approve every file change in the PR. If they have any problems with your changes, they will make comments on the PR on GitHub and you need to make the changes before they approve the PR. If they approve a file change, but you later decide to make another change to the same file, that file will need to be approved again.

## Merges

Once a PR is approved and is passing all CI/CD checks, that branch can be added to the merge queue. GitHub will automatically merge your branch into the main branch. If your branch is several commits behind main, it may not be able to be automatically merged. During development, you should be merging main into your branch whenever there are changes on main using `git merge main`. This ensures that any changes someone else has made don't conflict with your changes. Make sure you run `git pull` to update your local main branch before running this though, or there won't be any changes to be merged.

## Updating your local repo

The repo on your computer won't update automatically. To do this, you need to periodically run `git pull`, which updates all the branches from their counterparts on the GitHub. This won't get rid of branches that have been merged though. To do this, you can run `git remote prune origin`, which removes any branches on your local repo that have been deleted or merged on the GitHub. You can also combine these two using `git pull --prune`.
