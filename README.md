Git 全局设置:

git config --global user.name "hxz"
git config --global user.email "xz.h@foxmail.com"
创建 git 仓库:

mkdir dlo_ws
cd dlo_ws
git init
touch README.md
git add README.md
git commit -m "first commit"
git remote add origin https://gitee.com/huangxuzhao/dlo_ws.git
git push -u origin master
已有仓库?

cd existing_git_repo
git remote add origin https://gitee.com/huangxuzhao/dlo_ws.git
git push -u origin master
