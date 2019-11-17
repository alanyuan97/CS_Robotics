# Robotics

This repo is for [Robotics coursework](https://www.doc.ic.ac.uk/~ajd/Robotics/). The group size is five.

The project has continuous assessment every week.

## Workflow Management [draft]

### Commit message

Commit should start with

* __`BUG: `__ for bug fix
* __`WK{numer}: `__ for features related to assement in that week
* __`TOL: `__ for general productivity tools
* __`MIN:`__ for unimportant changes

Note that there is __a space__ after `:` for each per-fix. \
It then should be followed by short phrase describe what this commit is about.

### Branch setup

There will be four type of branches

* __master__ branch is for completed development
* __dev/{week_num}__ branch is for completed features for {week_num} week
* __fea/{week_num}/{task_name}__ branch is for incompleted feature for {week_num} week
* __tool__ branch is for general productivity tools

#### merge vs rebase

For cases below __merge__ should be used via pull-request and code review:

* `fea/{week_num}/{task_name}` to `dev/{week_num}`
* `dev/{week_num}`  to `master`, and at least one approval from another team member
* `tool` to `master`

For cases below __rebase__ should be used, to make history clear to read:

* pull from `master` to `dev/{week_num}`
* pull from `dev/{week_num}` to `fea/{week_num}/{task_name}`
* When pull from remote, use `git pull --rebase`

### User management

Even though we share only one raspberry pi as the brain for our robot. We should still create indicival account on the pi, such we could do modification simultaneously.

To achive this, there will be two type of account:

* __pi__ account for submission run, it should be keeped as clean environment. Not be used as dev account.
* __devs__ accounts for every teammate, it's for day to day develop stuff, no sudo it allowed.

### Dev environment

I would recommend everyone to VS Code with `Remote - SSH` extension. Set up git environment as you would on any Linux computer.
