#### 模式

命令模式、末行模式、编辑模式

#### vim进入

1、“vim 文件路径” 直接打开指定文件

2、“vim+数字 文件路径” 打开指定文件并将光标移动到指定行

3、“vim +/关键词 文件路径” 打开指定文件并高亮显示关键词

4、“vim 文件路径1 文件路径2 文件路径3” 同时打开多个文件，文件之间可切换操作

**部分文件修改时可能需要添加权限“sudo”**

#### vim退出

* “:q+回车”  退出当前文件
* “:wq+回车” 保存当前文件并退出
* “:q!+回车” 不保存当前文件修改并退出



#### 切换命令模式

* 用vim打开文件时默认为命令模式；

* 编辑模式下，按一下ESC可切换到命令模式；
* 末行模式下，按一下ESC或删除末行模式下的“:”可切换

#### 切换末行模式

* 命令模式下，输入“:”可切换到末行模式

#### 切换编辑模式

命令模式下输入字母i或a可切换到编辑模式



ps：以上为基本用法，更多可参考

https://blog.csdn.net/m0_73185293/article/details/131261754?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522be5dedafa03c57bea2a1f4cf860aa40e%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=be5dedafa03c57bea2a1f4cf860aa40e&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-131261754-null-null.142^v101^