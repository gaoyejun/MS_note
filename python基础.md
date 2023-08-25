# 1 基础
## 1.1 print输出
### 1.1.1 普通输出
+ 输出数字：print(123)
+ 输出字符串：print('qwe')或print("qwe")
+ 输出表达式: print(1+3)
>需要注意，直接使用print会自带换行，如果想不换行需要用逗号隔开每一段字符
>例如：printf('qqwe','asd')

### 1.1.2 输出到文件
~~~python
fp = open('D:/text.txt,'a+'') #a+表示如果文件不存在就创建，如果存在就在后方换行添加
print('hello word',file=fp)
fp.close()
~~~

### 1.1.3 转义字符
+ 反斜杠：\\
+ 单引号：\'
+ 双引号：\"
+ 换行：\n
+ 回车：\r
+ 水平制表符（TAB）：\t
+ 退格：\b	

## 1.2 数据类型
### 1.2.1 字符串类型
和C/C++类似，==但是字符串类型为”str“==
python里面可以用单引号、双引号、三引号三种方式定义字符串。==这样当你想打印的字符串里有双引号时，可以直接用三引号定义字符串，就不用转义字符了==
#### 1.2.1.1 字符串拼接
使用+加号，直接让两个字符串变量相加就行

还有一种是通过占位拼接的方式
~~~python
message = "i need %s，%s" % ("car","ok")
~~~
这里%s表示将后方的变量转换成字符串类型数据放在这里，变成"i need car,ok"
占位还有%d和%f，分别表示用后方的整数和浮点数替换这里
#### 1.2.1.2 字符串的格式化
使用  %5d   表示将数字宽度控制在5位；使用  %5.2f  表示将证书宽度控制在5位，小鼠保留两位；使用 %.2f表示只要求包保留小数点后两位
#### 1.2.1.3 更简洁的字符串拼接
例子如下
~~~python
mes = "car"
num = 123
print(f"i need {mes},price {num}")
~~~
注意不要忘了前面的"f"，这其中所有内容都是按照其本身数据类型转变的。输出为"i need car,price 123"

### 1.2.2 浮点类型
浮点类型存在计算精度问题，需要使用特殊手段保证运算精度
~~~python
from decimal import Decimal
print(Decimal('1.1')+Decimal('2.2'))
~~~
使用这个就可以保证浮点数的相加精度
### 1.2.3 数据类型转换
三个类型转换：str() int() float()
>有一点需要注意，如果要将字符串转为数字，必须保证字符串里都是数字的字符

## 1.3 常用运算符
+ （+）加
+ （-）减
+ （*）乘
+ （/）除
+ （//）**取商（整数）**，只返回运算的整数部分
+ （%）取余
+ （**）指数
+ 关系运算符和C/C++一样

## 1.4 使用小技巧
### 1.4.1 pass空实现
有时想先写出函数、类等代码的框架，还不知道里面具体些什么，可以这样写
~~~python
class Student:
	pass

stu = student()
~~~
这样写就可以在程序不报错的情况下，下吧代码框架搭好

# 2 语句
## 2.1 判断语句
判断语句是通过缩进块来判断哪些代码是if语句中的
### 2.1.1 if-else
~~~python
if 判断条件:
	语句1
	语句2
语句3
~~~
其中判断条件后的==:==不要忘记，并且==语句3不是if判断语句中的==，因为没有==4个空格==缩进，同理ifelse如下
~~~python
if 条件:
	语句
else:
	语句
	
if 条件:
	语句
elif 条件2:
	语句
else:
	语句
~~~
==注意，if语句的嵌套和C/C++一样，只需要注意缩进对齐就行==
### 2.1.2 if-in/if-not in
~~~python
kj = ['a','b','c','d','e']   #定义一个列表
if 'b' in kj:
    print("a在列表中")
    
if 'h' not in kj:
    print("h不在列表中")
~~~


## 2.2 while循环
基本格式
~~~python
while 条件:
	语句1
	语句2
	....
~~~
## 2.3 for循环
### 2.3.1 基本格式
基本格式如下
~~~python
for 临时变量 in 待处理的数据集:
	语句1
	语句2
	...
~~~
待处理的数据集是==序列类型==，比如字符串、列表、元组等。
~~~python
for i in "qweqwe":
	print(x)
~~~
这个代码效果就是输出"qweqwe"，for语句一个个取出字符执行print语句
### 2.3.2 range语句助力for循环
+ range(num)，生成0~num-1的序列，==这个序列不包含num==
+ range(begin,end),生成begin~end-1的序列，==这个序列不包含end==
+ range(begin,end,step),生成begin~end-1的序列，序列中两个数之间相隔step，==这个序列不包含end==
### 2.3.3 for训练临时变量作用域
代码如下
~~~python
for i in range(5):
	print(x)
print(i)
~~~
现在的版本即使在for循环执行完，临时变量i依旧可以被访问到，并且保留了之前执行for循环产生的变化

# 3 函数
## 3.1 基本格式
~~~python
def 函数名(传入参数):
	函数体
	return 返回值
~~~
函数必须先定义后使用
## 3.2 多种传参方式
### 3.2.1 位置参数
就是传统的参数传递方式，函数定义时形参顺序是什么，使用函数是实参顺序就得是什么
### 3.2.2 关键字参数
调用函数时语法具体形式是 ：函数名（形参1=实参1，形参2=实参2，......）
~~~python
def func(name,age):
	print(name,age)

#调用子函数
#关键字传参
func(name="小明",age=19)
#可以不按顺序
func(age=19，name="小明")
#可以混用，但是位置参数必须在前，且匹配参数顺序
func("小明",age=19)
~~~
==位置参数和关键字参数混用时，位置参数必须放在前面，但是关键字参数之间不存在先后顺序==
### 3.2.3 缺省参数
即函数有默认参数值

==默认参数必须放在参数列表最后==

~~~python
def func(name,age=18):
	函数体
	return 返回值

#使用子函数
func("小明")#这里就不用传递age参数了
func("小明"，20)#如果传递了，就使用传递的参数
~~~
### 3.2.4 不定长参数
**第一种**是位置传递

~~~python
def func(*age):
	函数体
	return 返回值
~~~
此时加了*号得形参==age是一个元组类型数据==，可以存放任意多的传入参数

**第二种**是关键字传递

~~~python
def func(**kwargs):
	函数体
	return 返回值
	
func(name="小明",age=18)
~~~
此时加了**号得形参==kwargs是一个字典类型数据==，可以存放任意多的传入键-值对

## 3.3 返回值
当不写return语句时，默认返回None，也可以手动返回None。==且None等效于False==

==None也可以赋值给变量==，表示这个变量后续会赋值，此处先暂时用None表示赋值一个无意义量

如果函数返回多个返回值，语法结构如下所示
~~~python
def func():
	return 1,2
	
#子函数返回值接收
x,y = func()
~~~
## 3.4 函数作为参数传递
~~~python
def func(compute):
	result = compute(1,2)
	return result
def compute(x,y)
	return x+y
	
func(compute)
~~~
## 3.5 lambda匿名函数
语法格式如下
~~~python
lambda 参数1，参数2:函数体（一行代码）
~~~
+ 匿名函数只能使用一次
+ 匿名函数内只能写一行代码
## 3.6 函数内部说明用注释
直接在函数体内部键入三引号对，然后一个回车就可以了，说明内容是自动补全的
~~~python
def myfuc(x,y):
    """

    :param x:
    :param y:
    :return:
    """
    return (x+y)

print(myfuc(1,2))
~~~

# 4 容器

## 4.1 list
==列表中存放的数据类型可以不一样，也可以嵌套==
### 4.1.1 list定义
列表的定义如下
~~~python
# 字面量
[元素1，元素2，元素3，...]
#定义变量
变量名 = [元素1，元素2，元素3，...]
#定义空列表
变量名称 = []
变量名称 = list()
~~~
### list下标索引
==第一个元素索引为0==
~~~ptyhon
my_list = ['a','b','c']
print(my_list[0]) #取出字符a
print(my_list[1]) #取出字符b
print(my_list[2]) #取出字符c
~~~
==也可以反向索引==，此时最后一个元素为-1
~~~ptyhon
my_list = ['a','b','c']
print(my_list[-1]) #取出字符c
print(my_list[-2]) #取出字符b
print(my_list[-3]) #取出字符a
~~~
嵌套的列表索引如下
~~~ptyhon
my_list = [['a','b','c'],'d']
print(my_list[0][1]) #取出字符b
~~~
### list的方法
**index(元素)**:查询列表元素的下标,==查询不到程序报错==
~~~python
index = mylist.index("hello")
~~~
**extend(其他数据容器)**:将其他容器中元素批量追加到list中
~~~python
mylist = [1,2,3]
mylist.estend([4,5,6])
print(mylist)  #输出结果为1,2,3,4,5,6
~~~
**append(元素)**:将元素添加到列表末尾
~~~python
mylist = [1,2,3]
mylist.append(4)
print(mylist)  #输出结果为1,2,3,4
~~~
**pop(下标)**:从list中取出元素，原list中元素被删除
~~~python
element = mylist.pop(2) #取出元素
~~~
**del 列表[下标]**:删除指定下标的元素
~~~python
del mylist[2]
~~~
**remove(元素)**:在列表中从前往后找，第一个匹配的元素被删除，==注意只能删除一个==
~~~python
mylist.remove('hrllo')
~~~
**clear()**:清空列表
~~~python
mylist.clear()
~~~
**count(元素)**:统计这个元素在列表中有几个
~~~python
count = mylist.count("hello")
~~~
**len(列表)**:统计列表中有多少元素
~~~python
count = len(mylist)
~~~

## 4.2 元组
==元组定义完成后不可修改==
==元组可以容纳不同类型元素==
### 4.2.1 元组的定义
~~~python
#字面量
(元素,元素,...)
#定义元组变量
变量名 = (元素,元素,...)
#定义空元组
变量名 = ()
变量名 = tuple()
~~~
特别注意，==如果定义的元组只有一个元素，必须在元素后加上逗号==
~~~python
mytuple = ("hello",)
~~~
### 4.2.2 元组的索引
和list一样，从0开始索引，支持使用[ ] 进行索引，也支持前后两个方向索引
### 4.2.3 元组的方法
**index(元素)**:查找元素对应的下标
~~~python
index = mytuple.index("hello")
~~~
**count(元素)**:统计元组中有多少个对应元素
~~~python
count = mytuple.count("hello")
~~~
**len(元组)**:统计元组中有多少元素
~~~python
count = len(mytuple)
~~~

## 4.3 字符串
字符串和元组一样，==也是一个不可修改的容器==
### 4.3.1 字符串的索引
和list一样，从0开始索引，支持使用[ ] 进行索引，也支持前后两个方向索引
### 4.3.2 字符串的方法
**index(字符/字符串)**:查找对应字符的起始位置，==如果查找的是字符串，就返回这个字符串第一个字符的起始位置==
~~~python
index = mystr.index("and")
~~~
**replace(字符串1，字符串2)**:将字符串内全部的字符串1替换为字符串2。不是修改字符串，是得到一个符合要求的新的字符串，==因此需要新的字符串变量去接收替换后的字符串==
~~~python
newstr = oldstr.replace("and","or") #注意必须找一个新的字符串去接收
~~~
**split(分隔符字符串)**:按照指定的分隔符字符串，将字符串划分为多个子字符串，并存入列表对象中。==字符串本身不变，而是得到了一个列表对象==,并且分隔符本身不会被编入子字符串中
~~~python
my_str_list = mystr.split(" ") #以空格为分隔符
~~~
**strip(无参数/字符串)**:不传入参数默认去除首尾空格，传入字符串就去除首尾中和传入字符串中字符相同的字符
~~~python
oldstr = "12i need cgar21"
newstr = oldstr.strip("12") #此时newstr = “i need car”
~~~
>注意传入字符串“12”，等同于去除首尾中的字符“1”和“2”

**count(字符串)**:统计字符串出现的次数
~~~python
count = mystr.count("and")
~~~
**len(字符串变量)**:统计字符串的长度
~~~python
num = en(mystr)
~~~
### 4.3.3 字符串的拼接
==这里拼接产生的是新字符串==
（1）`str1 + str2`:直接相加两个字符串
（2）`str = '我%d岁，永远%s岁' % (18,'18')`:用括号里的数据，按照转义字符和原字符串拼接成新的字符串
（3）
~~~C++
age = 18
str1 = '我{}{}岁,{}{}岁'.format('今年',age,'明年',age+1)
str2 = '我{2}{0}岁,{3}{1}岁'.format(age,age+1,'今年','明年')   #花括号中的是参数的位置索引
str3 = '我今年{age1}岁,明年{age2}岁'.format(age1 = 18,age2 = 19)
~~~


## 4.4 数据容器（序列）的切片
+ 序列：指内容连续、有序，可以使用下标索引的数据容器。==列表、元组、字符串都是序列==
+ 切片：指从序列中取出一个子序列

语法： 序列 [ 起始下标 ：结束下标 ： 步长 ]
> + 起始下标表示从何开始，留空表示从头开始
> + 结束下表表示截至在哪里，留空表示到末尾
> + 步长为1表示一个一个取；步长为2表示每次跳一个取；步长为N，表示跳过N-1个取
> + 步长为负数表示反向取
> + ==结束下标对应的元素不会取到==
> + ==切片不会影响序列本身==

## 4.5 集合
集合的最==主要特点==就是
+ 不允许重复元素
+ 内容无序
+ 不支持下标索引
### 4.5.1 定义
~~~python
#定义字面量
{元素，元素，...}
#定义集合变量
变量名 = {元素，元素，...}
#定义空集合
变量名 = set()
~~~
### 4.5.2 添加新元素
~~~python
my_set.add("hello")
my_set.add("hello") #这里虽然添加了两遍“hello”，但是集合里还是只有一个
~~~
### 4.5.2 移除元素
~~~python
my_set.remove("hello")
~~~
### 4.5.3 随机取出元素
~~~python
element = my_set.pop()
~~~
### 4.5.4 清空元素
~~~python
my_set.clear()
~~~
### 4.5.5 取两个集合的差集
~~~python
set1.difference(set2)
~~~
有以下几点注意
+ 取set1中有而set2中没有的元素
+ set1会被改变，set2不会被改变
+ ==相当于删除set1中和set2相同的元素==
### 4.5.6 合并两个集合
~~~python
set3 = set1.union(set2)
~~~
合并得到新集合，==原有集合不会改变==
### 4.5.7 统计集合元素数量
~~~python
num = len(my_set)
~~~
==不会统计重复的元素==
### 4.5.8 集合遍历
因为不支持下标索引，所以不能使用while，但是==可以使用for循环==

## 4.6 字典
### 4.6.1 定义
~~~python
#定义字面量
{key:value , key:value , ....}
#定义字典变量
变量名 = {key:value , key:value , ....}
#定义空字典
变量名 = {}
变量名 = dict()
~~~
+ 字典可以嵌套
+ ==key的类型不允许为字典==
### 4.6.2 定义重复key的字典
字典不允许key的重复，如果定义两个相同key的元素，则保留后定义的哪个元素
### 4.6.3 元素的访问
不允许下标索引，只能使用key来访问元素
~~~python
value = my_dict[key]
value = my_dict[key1][key2] #当字典出现嵌套时，用这种方式访问元素
~~~
### 4.6.4 字典修改操作
**新增/修改元素**：字典 [key] = value
### 4.6.5 字典的删除和清空
**取出并删除**：value = 字典 . pop(key)。将对应key的元素取出，并在字典中删除这个元素
**清空字典**：字典 . clear()
### 4.6.5 获取字典中全部的key
~~~python
keys = my_dict,keys()
~~~
==这里keys的变量类型属于dict_keys==
### 4.6.6 遍历字典
使用for循环，可以直接循环字典本身，也可以取出所有key，然后通过key遍历字典

字典不支持while循环
### 4.6.7 统计字典元素数量
~~~python
num = len(my_dict) 
~~~

## 4.7 容器通用操作
### 4.7.1 基本操作
+ **len(容器名)**：容器中元素个数
+ **max(容器名)**：容器中最大元素
+ **min(容器名)**：容器中最小元素

==容器中存放字符时，是按照ASCII表对应的值比较==
### 4.7.2 容器转列表
使用函数 list()。需要注意
+ 列表转列表、元组转列表、集合转列表 都是直接吧元素搬过去
+ 字符串转列表是将字符串中每个字符单独作为一个元素转过去
+ 字典转列表是将字典的key转过去
### 4.7.3 容器转元组
使用函数 tuple()。需要注意
+ 列表转元组、元组转元组、集合转元组 都是直接吧元素搬过去
+ 字符串转元组是将字符串中每个字符单独作为一个元素转过去
+ 字典转元组是将字典的key转过去
### 4.7.4 容器转字符串
使用函数 str()。需要注意
+ 列表转字符串结果是字符串，为“[元素，元素，...]”
+ 元组转字符串结果是字符串，为“（元素，元素，...）”
+ 字典转字符串结果是字符串，为"{key:value , key:value , .....}”
### 4.7.3 容器转集合
使用函数 set()。需要注意
+ 字符串转集合是将字符串中每个字符单独作为一个元素转过去
+ 字典转集合是将字典的key转过去
### 4.7.4 通用排序
使用语法结构为 ： ret = sorted(容器)
+ ==排序函数的返回值是列表类型数据==
+ 字典排序丢失value

反向排序的语法结构为 ： ret = sorted(容器，reverse=True)

# 5 文件操作
文件读写分三步走：打开文件、读写、关闭文件
## 5.1 文件打开
函数格式：open(name,mode,encoding)
+ name：打开的目标文件，可以包含具体路径
+ mode：设置打开文件的模式，只读'r'、写入'w'、追加'a'
+ encoding：编码格式，一般使用UTF-8
==如果写入的文件不存在，创建新文件==
~~~python
f = open("text.txt",'r',encoding="UTF-8")
~~~
## 5.2 文件读取
**文件对象.read(num)**：num表示要从文件中读取的数据长度（单位是字节），如果没有传入num，则表示读取全部数据
>==注意，如果之前使用read函数读取过一部分内容，再使用不传入num的read函数企图读取全部数据时，只会从上一个read结尾读取之前未读取完的数据==

**lins = 文件对象.readlines()**：读取文件中所有数据，变换成一个列表用返回值返回。文件中每一行变成列表中一个元素

**const = 文件对象.readline()**：一次读取一行数据

也可以使用for循环去依次读取文件中的每一行
~~~python
for line in f
	print(line)
~~~
## 5.3 文件关闭
**文件对象.close()**：关闭文件

## 5.4 文件的写入
使用open()函数打开文件时，模式选择'r'

**文件对象.write(字符串)**：文件写入，==但此时并没真正写入文件，只是存放在程序内存中，称之为缓冲区==，这里需要特别注意，==如果文件不存在，会创建文件；如果文件存在，会清空原有内容重新写入==

**文件对象.flush()**：将内容真正写入
## 5.5 文件的追加
使用open()函数打开文件时，模式选择'a'。其余函数也是使用wirte()和flush()

如果文件不存在，会创建文件；如果文件存在，则会在原有内容后写入新的数据

# 6 异常
提前考虑可能出现什么BUG，然后提前准备处理代码
## 6.1 捕获异常
基本语法（捕获所有异常）
~~~python
try:
	可能出现异常的代码
except:
	如果出现异常执行的代码
~~~
捕获指定异常（以变量未定义异常为例）
~~~python
try:
	可能出现异常的代码
except NameError as e:
	如果出现异常执行的代码
~~~
>这里的e是存放异常信息的一个变量，可以直接print，会输出异常信息

捕获多个异常（以变量未定义和除0异常为例）
~~~python
try:
	可能出现异常的代码
except （NameError，ZeroDivisinError） as e:
	如果出现异常执行的代码
~~~
捕获全部异常
~~~python
try:
	可能出现异常的代码
except Exception as e:
	如果出现异常执行的代码
~~~

异常else，是一个可选部分。即如果不发生异常执行的代码
~~~python
try:
	可能出现异常的代码
except Exception as e:
	如果出现异常执行的代码
else:
	如果没有异常执行的代码
~~~

异常finally，表示无论有没有异常都要执行的代码
~~~python
try:
	可能出现异常的代码
except Exception as e:
	如果出现异常执行的代码
else:
	如果没有异常执行的代码
finally:
	无论有没有异常都要执行的代码
~~~
## 6.2 异常的传递
一个函数调用的子函数中出现异常，如果子函数中没有捕获这个异常，则这个子函数中出现的异常会传递给调用子函数的函数中。不断往上传递，直到被捕获

# 7 模块/包
模块类似一个工具箱，里面包含类、变量、方法等。是一个以.py结尾的文件
## 7.1 导入模块
**import 模块名**：导入一个模块的全部
~~~python
import time
time.sleep(5) #导入time模块，使用其中的某个函数
~~~
**form 模块名 import 功能名**：导入一个模块中的某个功能方法
~~~python
form time import sleep
sleep(5) #可以直接使用sleep
~~~
**form 模块名 import * **：导入一个模块中的所有方法
~~~python
form time import *
sleep(5) #可以直接使用sleep
~~~
**form 模块名 as 别名 **  **form 模块名 import 功能名 as 别名 **：将导入的模块或者方法换一个别名
## 7.2 自定义模块
正常写py文件即可，用的时候直接import就可以用py文件中的内容。

+ ==注意，这里模块名就是py文件名==
+ 导入多个模块，且模块内有同名功能时，只会调用后导入的那个模块的功能

## 7.3 __main__ 变量
当在模块py文件中写了调用的语句时
~~~python
def func():
	return 1

func() #这里再模块文件中写了一个调用
~~~
当使用import导入这个方法时，会自动执行”func()“这条语句。为了避免这个情况，可以再模块py文件中这样写
~~~python
def func():
	return 1

if __name__ == '__main__' #这里判断是否是主函数调用
	func() 
~~~
加了上述语句，再外部导入模块时不会执行”func()“这条语句

## 7.3 __all__ 变量
~~~python
__all__ = ['funcA']

def funcA()
	return 1

def funcB()
	return 2
~~~
当模块py文件中使用了上述all语句时，当时使用“**from xxx import * **”时，只会导入funcA这个方法

## 7.4 python包
python包本质上就是文件夹，在该文件夹下包含了`__init__.py`文件，则这个文件夹就是python包。

包中可以使用all变量等手段，去设置模块的调用

导入包的方法：
1. `import 包名.模块名`，使用时使用如下语法`包名.模块名.目标`
2. `form 包名 import 模块名`,导入包中具体某个模块，使用方法`模块名.目标`
3. `form 包名。模块名 import 目标`,导入某个具体目标，可以直接使用目标

## 7.5 第三方包安装
`pip install 包名称`：在命令提示符窗口输入本指令安装第三方包

不过默认链接的服务器在国外，因此需要链接在国内的网站，使用指令`pip install -i https://pypi.tuna.tsinghua.edu.cn/simple 包名称`

另一个源

```text
pip install -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com
```

# 8 可视化图表
## 8.1 json数据格式
本质是一个带有特定格式的字符串。按照json指定格式去组织封装数据，就可以让数据在不同种语言之间进行传递

json的格式可以这么描述
+ 吧python中的字典的字面量格式直接转成字符串
+ 吧python中的列表（元素是字典）的字面量格式直接转成字符串

使用json数据的方法如下
~~~python
import json

#准备符合json格式的python数据
data=[{“name":"老王","age":16},{“name":"张三","age":17}]

#将python数据变为json数据
data = json.dumps(data)
data = json.dumps(data,ensure_ascii=False) #如果数据里包含中文，使用这句可以让中文显式的编码

#将json数据变为python数据
data = json.loads(data)
~~~

## pyecharts实现可视化功能
可以登录网站`https://gallery.pyecharts.org`去了解每一个绘图类型的具体操作

# 9 类
==特别注意，在python中，类对象是作为引用 传递给函数的。这意味着在函数内部修改类对象会影响到原变量==
## 9.1 类定义
~~~python
class 类名称:
	类属性（成员变量） = None
	类行为（成员方法）
	
#创建类对象
对象名 = 类名称（）
~~~
==注意类属性（成员变量）后面要加一个=None。==

## 9.2 成员函数
在类内部定义成员函数格式如戏曲
~~~python
class myclass:
	def func(self,形参1,....)
		return ....
~~~
需要注意
+ 定义成员函数时,`self`必须写，它就相当于this指针
+ 在使用类对象时，`self`是自动传入，不用手动写
+ ==在方法内部想要访问成员变量，必须使用self==
## 9.3 构造方法（构造函数）
~~~python
class myclass
	#如果写了构造方法，那么这里都可以不用写成员变量的定义
	name = None
	age = None
	
	def __init__(self,name,age)
		self.name = name
		self.age = age

# 构建类对象
my = myclass("tom",20)
~~~
==如果写了构造方法，在定义类时可以不在开头写成员属性的定义==
## 9.4 魔术方法（类似重载）
用双下划线包起来的函数都叫”魔术函数“吗，前文的构造方法也算。

**`__str__`字符串方法**

~~~python
class student:
	def __init__(self,name,age)
		self.name = name
		self.age = age
	def __str__(self)
		return f"名字是{self.name}，年龄是{self.age}"
	
Stu = student("tom",20)
print(Stu) #执行这句话就会调用__str__(self)这个函数
~~~

**`__lt__`小于符号比较方法**
~~~python
class student:
	def __init__(self,name,age)
		self.name = name
		self.age = age
	def __lt__(self,other)
		return self.age < other.age
	
Stu1 = student("tom",20)
Stu2 = student("tnnd",22)
print(Stu1<Stu2) #结果为 True
~~~

**`__le__`小于等于符号比较方法**
~~~python
class student:
	def __init__(self,name,age)
		self.name = name
		self.age = age
	def __le__(self,other)
		return self.age <= other.age
	
Stu1 = student("tom",20)
Stu2 = student("tnnd",20)
print(Stu1<=Stu2) #结果为 True
~~~

**`__eq__`小于等于符号比较方法**
~~~python
class student:
	def __init__(self,name,age)
		self.name = name
		self.age = age
	def __eq__(self,other)
		return self.age == other.age
	
Stu1 = student("tom",20)
Stu2 = student("tnnd",22)
print(Stu1==Stu2) #结果为 False
~~~

## 9.5 私有成员
+ 在成员变量或者成员函数前面加上**双下划线**可以将成员设置成私有
+ 私有成员在类外不可访问
+ 私有成员可以被本类内方法使用
~~~python
class stdent:
	__age = None #是私有
	name = None #不是私有
	
	def __my_age(self): #是私有
		print(self.age)
	def my_name(self):	#不是私有
		print(self.name)
~~~

## 9.6 继承
继承语法
~~~python
class 类名(父类名1,父类名2)
	def __init__(self):
		super(类名,self).__init__()	#这句是调用父类构造函数
	类内内容
~~~
==如果父类1和父类2中有同名成员，则保留父类1中的成员，及保留靠左的继承==

## 9.7 复写/调用 父类中同名成员
在子类继承父类成员后，直接在子类中按照相同名字直接重新写一遍父类中成员，就达到复写效果。注意：==成员属性也可以复写==

在子类中复写了父类成员后，还想调用父类成员，方法如下

方式1：
+ 通过类名调用父类成员
1. 使用成员变量：父类名.成员变量
2. 使用成员方法：父类名.成员方法（self）。==注意这个self不可省略==

方式2：
+ 使用super()调用父类成员
1. 使用成员变量：super().成员变量
2. 使用成员方法：super().成员方法()

## 9.8 多态
同样的行为/函数，传入不同的对象，得到不同状态

通常用在子类、父类继承关系中：
+ 父类来确定有哪些方法----------------【顶层设计】
+ 子类来具体实现（复写）这些方法----【具体实现】

这种写法就叫做**抽象类**（接口）。定义如下
+ **抽象类**：含有抽象方法的类
+ **抽象方法**：方法是空实现（pass）的
~~~python
class Animal:
    def speak(self):
        pass

class Cat(Animal):
    def speak(self):
        print("喵")

class Dog(Animal):
    def speak(self):
        print("汪")

def ANIMAL_SPEAK(Ani:Animal):
    Ani.speak()

cat = Cat()
dog = Dog()
ANIMAL_SPEAK(cat)#输出喵
ANIMAL_SPEAK(dog)#输出汪
~~~
# 10 注解
注解的目的是为自己写的变量、自定义类型提供注解，使得可以自动对参数的类型进行注解

==按下Ctrl+p可以查看注解==

## 10.1 变量的类型注解
基础语法：
	`变量:类型`
~~~python
#基础数据注解
var_my:int = 10

#类对象类型注解
class Student:
	pass
stu: Student = Student()

#对容器进行简单注解
my_list: list = [1,2,3]

#对容器进行详细注解
my_list:list[int] = [1,2,3]
my_tuple:tuple[str,int,bool] = ("dsf",666,True)
my_set:set[int] = {1,2,3}
my_dict:dict[str,int] = {"itin":666,"sdf":123}
~~~
+ 元组需要将每一个元素类型标记
+ 字典需要将key和value的类型都标记

==还可以在注释中注解==。语法是`#type:类型`

~~~python
var_my = 10	#type:int
var_my = {"itin":666,"sdf":123}	#type:dict[str,int]
~~~
## 10.2 函数（方法）的类型注解
可以对形参和返回值进行类型注解
~~~python
def add(x:int , y:int) -> int:
	return x+y
~~~
上述代码中->是对返回值的注解
## 10.3 Union联合注解
~~~python
from typing import Union

my_list: list[Union[str,int]] = [1,2,"dfs"]
my_dict: dict[str,Union[str,int]] = {"name":"tom" , "age":31}
~~~
上述代码表示在list中，数据要么是str要合适int；在字典中，value的数据类型要么是str要么是int

==函数也可以使用Union注解，格式和之前一样==