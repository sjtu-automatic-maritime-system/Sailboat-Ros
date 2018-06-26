检查：
gitk

添加：
git add .
git add --all

现状：
git status -s
git status

上传：
git commit -m 'test'
git push origin master

恢复：
git log
git rm --cached <文件>
git reset HEAD <文件>
git reset --hard HEAD^

下载：
git pull

改动：
git diff                尚未缓存的改动
git diff --cached       查看已缓存的改动
git diff HEAD           查看已缓存的与未缓存的所有改动
diff：git diff --stat   显示摘要而非整个

查看当前配置有哪些远程仓库：
git remote

clone:
git clone hywel@192.168.1.151:/home/hywel/sailboat-ws/src/Sailboat-Ros/.git
