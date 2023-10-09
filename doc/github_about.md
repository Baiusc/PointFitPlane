
# 1、检查自己git config是否配置
首先通过命令行`git config --global --list`查看自己的`git config`

如果没有`user.name`和`user.email`没有值的话，我们也可以先配置，命令行配置如下
```bash
git config --global user.name "这里换上你的用户名"
git config --global user.email "这里换上你的邮箱"
```
当然，你也可以在电脑内查看文件，以windows电脑为例，路径大概是`C:\Users\Administrator`，之后的ssh key也是生成在这个目录下。

# 2、新建ssh key
进入路径`C:\Users\Administrator`，命令行输入如下命令，执行生成你的sshkey
```bash
ssh-keygen -t rsa -C "这里换上你的邮箱"
ssh-keygen -t rsa -C "1148635540@qq.com"
```

回车后会询问你ssh key生成的路径、是否需要密码，不需要的话直接留空回车即可。

生成成功后，该路径下会出现如下文件

id_rsa.pub就是公钥文件，后续会使用

# 3、GitHub关联ssh
进入GitHub的个人设置，找到`SSH and GPG keys`，然后点击`New SSH key`，进入如下界面，`title`输入你对于当前SSH key的备注，下面的`key`就粘贴上一步生成的`id_rsa.pub`内的内容

操作完成你就能开心的使用SSH模式操作git了。

# 4、使用SSH
以GitHub为例，我们在查看code的时候如下图，使用SSH就好了

如果是之前项目用的HTTPS，现在要改成SSH，进行上面的操作后，我们可以直接切换origin的地址

```bash
git remote set-url origin git@github.com:你的仓库.git
```


# 5、命令行例子
```bash
git config --global --list
git config --global user.name bzs_work_pc
git config --global user.name bzs_own_pc
git config --global user.email baizhongshan@51creation.com
git config --global user.email 1148635540@qq.com
# 添加远程仓库
git remote add github git@github.com:Baiusc/CMakeProject1.git
git remote add gitee git@gitee.com/Baiusc/SeamRecognition.git 
# 查看已添加的远程仓库
git remote -v
# 生成 RSA 公钥私钥，双引号里的是对应的 GitHub 或者 Gitee 的邮箱地址
ssh-keygen -t ed25519 -C "xxxxx@outlook.com"
ssh-keygen -t rsa -C baizhongshan@51creation.com
# 命令行验证 GitHub 和 Gitee 的公钥
ssh -T git@github.com
ssh -T git@gitee.com
# 同时将代码推送到 GitHub 和 Gitee 的默认分支
git push --all github
git push --all gitee
# 推送标签（tags）【指定分支】到 GitHub 和 Gitee
git push --tags github
git push --tags gitee
```
配置公钥，打开 C:\Users\Administrator\.ssh 目录下，找到 .pub 的公钥打开，然后配置到自己的 GitHub 和 Gitee 上即可

# 6、清除Git缓存

```bash
git rm --cached -r .
git add .
git commit -m "提交项目文件，首次提交"
git push -u origin master
```
`git rm --cached -r .`这个命令将从`Git`缓存中删除当前目录及其所有子目录中的所有文件。`--cached` 参数表示仅在缓存中删除文件，而不会删除实际文件。`-r` 参数表示递归地删除子目录中的文件。

`git add .` 这个命令将重新将当前目录及其所有子目录中的所有文件添加到 Git 中。这些文件之前已经被 Git 跟踪过，但由于在`.gitignore` 文件中指定了忽略规则，因此需要重新添加。

# 7、建立仓库
```bash
echo "# CMakeProject1" >> README.md
git init
git add README.md
git add .
git commit -m "提交项目文件，首次提交"
git branch -M main
git remote add origin git@github.com:Baiusc/PointFitPlane.git
git push -u origin main
```
# 8、git同步提交
找到.git下面的config文件进行修改
```bash
[core]
	repositoryformatversion = 0
	filemode = false
	bare = false
	logallrefupdates = true
	symlinks = false
	ignorecase = true
[remote "origin"]
	url = git@github.com:Baiusc/CMakeProject1.git
	url = git@gitee.com:Baiusc/CMakeProject1.git
	fetch = +refs/heads/*:refs/remotes/origin/*
[branch "main"]
	remote = origin
	merge = refs/heads/main
```



