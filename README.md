Website
=======

Repository for hosting content on the SQUIRREL website (http://squirrel-project.eu/)

# Publications/Papers
Please DO NOT ENTER your PUBLICATIONS MANUALLY to the website. We have installed an automatic publications website generator script in the management package that will overwrite the publications website with the bibtex information provided there.
Please consider the instructions in the main readme.md file of the management package on adding new publications.

# Quick installation

Clone the squirrel-project website repository (or preferably your own fork of it)
```bash
git clone git@github.com:squirrel-project/website
```

Then simply edit the html files. For testing run:
```bash
website/debug_server.py
```

You can now browse to the url http://localhost:9000 and get a live update whenever you edit any html file located in squirrel_website/html.

# Installation

Create a new catkin workspace for the squirrel project website:
```bash
mkdir -p ~/squirrel_ws/src
cd ~/squirrel_ws/src
catkin_init_workspace
```

Clone the squirrel-project website repository:
```bash
git clone git clone git@github.com:squirrel-project/website
```

Add the workspace to the package path and install dependencies:
```bash
export ROS_PACKAGE_PATH=~/squirrel_ws:$ROS_PACKAGE_PATH
```

Install required dependencies:
```bash
rosdep install squirrel_website
```

If you want to permanently add the workspace to your repository path you can source the package path in your bashrc:
```bash
echo "export ROS_PACKAGE_PATH=~/squirrel_ws:$ROS_PACKAGE_PATH" >> ~/.bashrc
```


# Local Testing / Editing

Stat a local server:
```bash
roslaunch squirrel_website server.launch
```

You can now browse to the url http://localhost:9000 and get a live update whenever you edit any html file located in squirrel_website/html.


# Remote updating

To update the official website http://www.squirrel-project.eu, send a pull request to ipa320. It takes about 5-10 minutes to update the official website after the pull request was accpeted.
