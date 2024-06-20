# AUVIC Github Guidelines
---

### Workflow
1. Create or choose an existing issue and assign yourself
2. Create a branch associated with that issue
3. Work on the issue within the branch
4. Submit a pull request
5. If changes are requested upon review, make those changes
6. Once the pull request is approved, complete the merge and delete the old branch

---

### Git
**Local Workflow**
```
git pull
git checkout <working-branch>
```
Make your changes
```
git add <file(s)>
git commit -m "good commit message"
git push
```

---

### Issues
**Creating an Issue**
- Create a new issue by selecting the "Issues" tab, then "New Issue"
- Give the issue a clear and concise title
- Outline what the issue will change in the description
- Give the issue appropriate labels
- Add the issue to the "Software" project

---

### Branches
**Creating a Branch**
- Create a new branch by selecting the branch dropdown and typing the name of the new branch
- Give the branch a name following the below format:
    - <issue number>-<issue-title>
    - Eg. 4-create-peripheral-test

---

### Pull Requests
**Opening a Pull Request**
- Under the “Pull requests” tab, Select “New pull request”
- Select “Create pull request”
- In the description, include any differences between the issue and work done
- Under “Development” add the issue that the pull request will be closing

---

### Commits
**Good Practice**
- Keep commits small, they're easier to trace issues
- Be descriptive with your commit messages, other people need to know what changed
