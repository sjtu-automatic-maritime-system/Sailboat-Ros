git add .
git add --all

git diff

git rm --cached <文件>
git reset HEAD <文件>

git status -s
git status

git commit -m 'test'
git push 

git reset --hard HEAD^

git pull

尚未缓存的改动：git diff
查看已缓存的改动： git diff --cached
查看已缓存的与未缓存的所有改动：git diff HEAD
显示摘要而非整个 diff：git diff --stat

要查看当前配置有哪些远程仓库，可以用命令：git remote
