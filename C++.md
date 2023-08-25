# 1 基础
## 1.1 数据类型
### 1.1.1 枚举
枚举的定义格式如下`enum <类型名> {<枚举常量表>};`

举例：
~~~c++
enum color_set1 {RED, BLUE, WHITE, BLACK}; // 定义枚举类型color_set1
enum week {Sun, Mon, Tue, Wed, Thu, Fri, Sat}; // 定义枚举类型week

enum fruit_set {apple, orange, banana=1, peach, grape};
//枚举常量apple=0,orange=1, banana=1,peach=2,grape=3。

enum week {Sun=7, Mon=1, Tue, Wed, Thu, Fri, Sat};
//枚举常量Sun,Mon,Tue,Wed,Thu,Fri,Sat的值分别为7、1、2、3、4、5、6。
~~~
需要注意的是：
+ "枚举常量"或称"枚举成员"，是以标识符形式表示的整型量
+ 默认状态下，枚举成员对应的整数就是所列举元素的序号，序号从0开始。

枚举的使用：
~~~c++
enum color_set1 {RED, BLUE, WHITE, BLACK} color1, color2;
//color_set1 color1, color2;	也可以这么定义枚举变量
enum color_set2 { GREEN, RED, YELLOW, WHITE} color3, color4;

color3=RED;           //将枚举常量值赋给枚举变量
color4=color3;        //相同类型的枚举变量赋值，color4的值为RED
int  i=color3;        //将枚举变量赋给整型变量，i的值为1
int  j=GREEN;         //将枚举常量赋给整型变量，j的值为0
~~~

# 2 类和对象
C++面向对象的三大特性：==封装、继承、多态==
==类==是变量声明中的变量类型，==对象==指的是通过类创建的变量
## 2.1 封装
### 基础
类设计时，可以吧属性和行为放在不同的权限下加以控制
访问权限有三种：
+ public:类内和类外都可访问
+ protected:类内可以访问，类外不可以
+ private:类内可以访问，类外不可以
~~~c++
class lizi
{
	public:
	...;
    ...;
	protected:
	...;
    ...;
	private:
	...;
    ...;
};
~~~
>private这几个权限声明顺序和次数没有限制，可以改换顺序和多次重复
>可以以不写权限类型，此时默认权限是private

### class和struct的区别
class默认权限private，struct默认权限是public

### 类内函数的类外实现
类内可以只声明函数，函数的定义放在类外实现。好处是如果一个类中的成员函数很复杂，可以在头文件中写类的声明，另外的C文件写函数实现，提高类声明的可读性
~~~c++
class temp
{
public:
	void func();
}

void temp::func()
{
	...;
}
~~~

## 2.2 对象的初始化和清理
### 构造函数和析构函数
构造函数用来初始化，析构函数用来清理。
如果程序员不写，编译器会自动提供，但是是空实现
这两个函数不用主动调用，由编译器自动调用

构造函数语法：类名(){}
+ xxxxxxxxxx delete[] p;c++
+ 函数名就是类名
+ 可以有参数，可以重载
+ 在程序==调用对象==时自动调用，且只会调用一次

析构函数语法：~类名(){}
+ 没有返回值，不写void
+ 函数名就是类名前面加~
+ 不可以有参数，不可以重载
+ 在程序==对象销毁前==自动调用，且只会调用一次。比如生名在子函数里的对象，在子程序执行完自动销毁时执行析构函数

例子：
~~~c++
class person
{
	//构造函数
	person()
	{
		....;
	}
	//析构函数
	~person()
	{
		...;
	}
};
~~~

#### 构造函数的分类及调用
分为“有参构造函数”、“无参构造函数”；“普通构造函数”、“拷贝构造函数”

下面展示无参、有参、拷贝构造函数
~~~c++
class person
{
	public:
	int age;
	
	person()//无参
	{
		...;
	}
	person(int)//有参
	{
		...;
	}
	person(const person &p)//拷贝，这里用常量引用防止被拷贝对象被修改
	{
		age = p.age;
	}
};
~~~
调用构造函数有三种方法：括号法、显示法、隐式转换法

下面展示如何括号法调用构造函数

~~~c++
person p1;//默认构造函数调用
person p2(10);//有参构造函数调用
person p3(p2);//拷贝构造函数调用
~~~
下面展示如何使用显示法调用构造函数
~~~c++
person p1;
person p2=person(10);
person p3=person(p2);
~~~
>person(10)这种形式的叫匿名对象，当前行执行完后及释放

下面展示隐式转换法
~~~c++
person p2=10;
person p3=p2;
~~~

#### ==拷贝构造函数的调用时机==
有三个时机：创建一个对象时调用、值传递的方式给函数参数传值、值方式返回局部对象

下面展示“值传递的方式给函数参数传值”
~~~c++
void dowork(person p)
{
	....;
}
void main()
{
	person p1;//person是一个类
	dowork(p1);//这里吧 p1作为一个实参传递给子函数的形参，这一过程是值传递，会用p1拷贝初始化形参p
}
~~~

下面展示“值方式返回局部对象”
~~~c++
person dowork()
{
	person p;
	return p;//此时返回的是子函数中局部变量p的副本，因此返回去的时候有拷贝构造发生
}
~~~

#### 拷贝构造函数的调用规则
如果你写了有参构造函数，编译器会默认提供拷贝构造函数，但不会提供默认构造函数

但是如果你写了拷贝构造函数，则==编译器不会提供任何其他构造函数==

#### 深拷贝和浅拷贝
这个问题会出现在==类中存在堆区内存申请时==，此时用编译器默认提供的拷贝构造函数会出现内存==重复释放==的现象

程序中存在如下代码
~~~c++
class person
{
	public:
	person(int height)
	{
		m_Height = new int(height);
	}
	
	~person()
	{
		if(m_Height!=NULL)
		{
			delete m_Height;
			m_Height = NULL;
		}
	}
	
	int *m_Height;
};
~~~
此时如果进行如下初始化
~~~c++
person p1(10);
person p2(p1);
~~~
则会出现报错，需要在类中自己写一个深拷贝，部分代码如下
~~~c++
person (const person &p)
{
	m_Height = new int(*p.m_Height);
}
~~~

#### 初始化列表
实现的功能是初始化类的属性，格式如下
~~~c++
class templet
{
	public:
	int a,b,c;
	
	templet():a(1),b(2),c(3)
	{
		
	}
};

void main()
{
	templet temp;
}
~~~
上述代码在初始化变量temp时，就把该变量的属性a,b,c初始化了。

为了能够改变初始化的值，也可以这样写
~~~c++
class templet
{
	public:
	int a,b,c;
	
	templet(int aa,int bb,int cc):a(aa),b(bb),c(cc)
	{
		
	}
};

void main()
{
	templet temp(1,2,3);
}
~~~


#### 类对象作为类成员
假设B这一类中包含新的类A，则初始化采用列表初始化
~~~c++
class student
{
public:
	student(string nam, string ph) :name(nam),pho(ph)
	{
		
	}
	string name;
	Phone pho;
};
~~~
==且调用构造函数时，A先构造，B后构造。调用析构函数时，B先析构，A再析构。==

### 静态成员
静态成员就是指成员变量或成员函数前加上static关键字
####  静态成员变量
+ 所有对象共享一份数据(即其他类修改这个静态成员变量后，其他类对象再访问时结果是修改后的)
+ 在编译阶段分配内存
+ 类内声明、类外初始化

下面展示类内声明、类外初始化操作（不类外初始化会报错）
~~~c++
class person
{
	static int a;
};
int person::a=100;
~~~
==静态成员变量不属于某个对象，所有对象都共享同一份数据==
==因此有两种访问方式==

+ 通过对象进行访问：p.a
+ 通过类名进行访问：person::a

==静态成员变量有访问权限，加了权限的类外不能访问（除了在类外进行初始化定义）==

#### 静态成员函数
+ 所有对象共享一个函数
+ ==静态成员函数只能访问静态成员变量==
~~~c++
class student
{
	public:
	static void func()
	{
		...;
	}
};
~~~

访问静态成员函数有两种方法
+ 通过对象进行访问：p.func()
+ 通过类名进行访问：person::func()

==静态成员变量有访问权限，加了权限的类外不能访问==

## 2.3 C++对象模型和this指针
### 类的数据和类的函数在内存地址上是分开存储的

每一个空的类对象也是有一个内存空间的（1字节）

考虑如下代码
~~~c++
class A
{
	int temp;
};
class B
{
	int temp;
	static int temp2;
};
int B::temp2=0;
class C
{
	int temp
	static void func()
	{
		...;
	}
};

void main()
{
	A a;
	B b;
	C c;
}
~~~
以上三个变量a,b,c虽然所属的类不同，但是使用sizeof查询其占用的内存空间后发现，这三个对象占用的存储空间都是4个字节，也就是说静态的成员变量和函数都和类中的其他元素不在一个内存空间上。

### this指针
每个类的非静态成员函数只会有一份函数实例，也就是说多个同类的对象会共用同一快代码，这块代码如何区分是哪一个对象调用？

使用this指针达到上述效果，每个this指针指向被调用的成员函数所属的对象。
指针不用定义，直接使用
+ 如果成员属性和成员函数的形参同名，用this指针区分
+ 如果要返回对象本身，可以用this指针
~~~c++
class test
{
	public:
	void func(int a)
	{
		a=a;//错误的，这里相当于吧形参a的值又赋给了形参a
	}
	
	int a;
};

class test
{
	public:
	void func(int a)
	{
		this->a=a;//正确的，吧形参a的值赋值给了成员变量a
	}
	
	int a;
};
~~~
~~~c++
class person
{
public:
	int age;
	person& addage(int age)//注意这里的返回类型必须是引用，不然返回的不是对象本体，而是和对象本体一摸一样的一个拷贝
	{
		this->age+=age;
		return *this;
	}
};

void main()
{
	person p1(10);
	person p2(10);
	
	p2.addage(p1);//此时p2中成员变量age的值是20
	
	p2.addage(p1).addage(p1);//此时p2中成员变量age的值是40
	//原因是第一次调用addage函数后，返回值是一个对p2原对象的引用，因此可以继续加一个点使用addage函数
}
~~~

### 用空指针调用成员函数
例子如下
~~~c++
class test
{
	public:
	void func()
	{
		cout<<"haha"<<endl;
	}
};

void main()
{
	test *P=NULL;//让对象P指向空指针
	
	P->func();
}
~~~
上面的代码是可以正常运行的，因为即使P是空指针，但是==成员函数func()里也没用任何其他成员属性==，因此空指针不影响func()的执行

如果将类声明改成如下的代码就不能正常执行了，会报错
~~~c++
class test
{
	public:
	char a;
	void func()
	{
		cout<<"haha"<<a<<endl;
	} 
};
~~~
但是进行如下的修改则可以不报错，虽然也不能达到预期效果
~~~c++
class test
{
	public:
	char a;
	void func()
	{
		if(this==NULL)
		{
			return;
		}
		cout<<"haha"<<a<<endl;
	}
};
~~~

### const修饰成员函数
常函数：
+ 加了const的成员函数就是常函数
+ 常函数不可修改成员属性
+ 成员属性增加关键字mutable后，在==常函数和常对象==中可以修改

常对象
+ 加了const的类对象成为常对象
+ 常对象只能调用常函数

常函数声明
~~~c++
class test
{
	void func() const
	{
		...;
	}
};
~~~
常函数内不可修改成员属性
~~~c++
class test
{
	int a;
	void func() const
	{
		a=1;//错误
		//这里的语句相当于：this->a=1;
	}
};
~~~
>基本原理是，不加const之前，调用成员函数时，默认this指针的声明是这样的“test * const this”
>而加了const修饰为常函数后，this指针的声明相当于'const test * const thsi'
>前者是顶层常量指针，指向对象可修改；后者是底层+顶层常量指针，指向对象不可修改

可以这样修改防止错误
~~~c++
class test
{
	mutable int a;//有了修饰词
	void func() const
	{
		a=1;//正确
	}
};
~~~

## 2.4友元
友元的目的是让一个==函数或者类==，访问另一个类中的私有成员
关键字为 friend
### 全局函数做友元
~~~c++
class room
{
	friend void func(room & rom);//如果没有这句话，main函数中func的调用就会报错
private:
	int room_num;
}

void func(room & rom)
{
	rom.room_num=1;
}

void main()
{
	room ROOM;
	func(ROOM);
}
~~~
### 类做友元
~~~c++
class person
{
	friend class biluding;//表示biluding这个类也可以访问私有成员变量num
private:
	int num;
}
~~~
### 成员函数做友元
~~~c++
class building
{
	friend void person::func();//表示类person中的成员函数func()可以访问私有成员变量num
private:
	int num;
}
~~~
-----------
## 2.5 重载
### ==运算符重载的本质==

运算符重载本质上是函数的重载，这个函数就是operator。只不过这个函数的调用可以简写成运算符的形式。

既然是函数重载，也就是说重载运算符时，参与的对象不一定必须都是类对象，也可以是int、char等，可以实现类与类之间的运算，也可以实现类与其他类型数据的运算。

关于类内重载和类外重载，本质上没有区别，只不过是重载函数定义的位置是类内还是类外。类内重载默认有一个参数是当前类，并且处于运算符左端。

函数调用运算符重载只能类内重载

-----
### 加号运算符重载
**使用成员函数重载**
~~~c++
class person
{
public:
	person operator+(person& p)
	{
		person temp;
		...;
		return temp;
	}
}

void main()
{
	person p1,p2;
	person p3=p1+p2;//等效于下面的指令
	person p3=p1.operator+(p2);//是成员函数重载的本质
}
~~~
**使用全局函数重载**
~~~c++
person operator+(person& p1,person& p2)
{
	person temp;
	...;
	return temp;
}

void main()
{
	person p1,p2;
	person p3=p1+p2;//等效于下面的指令
	person p3=operator+(p1,p2);//是全局函数重载的本质
}
~~~
-----------
### 左移运算符重载
**使用成员函数重载**
拟实现效果为“cout<<P”，其中P是一个类对象
~~~c++
class person
{
public:
	void operator<<(auto cout)
	{
		...;
	}
}
~~~
上述操作本质等效于
~~~c++
p.operator<<(cout)
p<<cout
~~~
因为这样的格式只能让类对象在左，cout在右，很固定，==因此在实践中，左移运算符重载经常不用成员函数重载==

**使用全局函数重载**
~~~c++
ostream& operator<<(ostream& cout,person& P)
{
	...;
	return cout;
}

void main()
{
	person P;
	cout<<P<<endl;//等效于下面的指令
	operator<<(cout,P)<<endl;//是全局函数重载的本质
}
~~~
-----------
### 递增（递减）运算符重载
分为前置/后置递增（递减）运算符
~~~c++
class person
{
	//前置,++P
	person& operator++()
	{
		m_age++;
		return *this;
	}
	//后置,P++
	//后置与前置的区分利用了函数重载，通过使用占位参数来区分，必须是int占位
	person operator++(int)
	{
		int temp=m_age;
		m_age++;
		return temp;
	}

private:
	int m_age;
}
~~~
>前置返回的是引用，后置返回的是值
>返回类对象的引用是为了能够链式的前置递增（递减）
>后置递增不能链式的使用，因为从逻辑上来说，后置递增返回的是已经不存在的历史对象，既然已经不存在了，自然==不能当左值==，只能当右值，==这一点和C自带的后置递增（递减）一样，返回的都不是左值==

----------
### 赋值运算符重载
==**赋值运算符重载**和构造函数、析构函数、拷贝构造函数一样，都是定义类时编译器默认生成的，多在出现深浅拷贝问题时重载==

深浅拷贝指的是在类初始化时，向堆区申请了空间，返回了指定的指针指向这个空间。在类赋值时，究竟是吧指针指向的值返回，还是吧指针返回。

深浅拷贝问题详见构造函数和析构函数那一节，本节只说明赋值运算符重载在深浅拷贝中的问题
~~~c++
class person
{
public:
	person()
	{
		m_age = new int (18);
	}
	~person()
	{
		if(m_age!=NULL)
		{
			delete m_age;
			m_age = NULL;
		}
	}
	person& operator=(person& P)
	{
		if(m_age!=NULL)
		{
			delete m_age;//如果拷贝前，这个类对象已经指向堆区了，就先释放了堆区的
		}
		
		m_age = new int (*p.m_age);//深拷贝操作
        
        return *this;//目的是为了能够连续的使用赋值运算
	}
	int* m_age;
}

int main()
{
	person P1,P2;
	P2 = P1;//如果不将赋值运算符重载，这一步会产生错误
    person P3;
    P3 = P2 = P1;//如果赋值运算符重载时返回值不是类引用则无法实现连续赋值
}
~~~
----------
### 关系运算符重载
关系运算符包括“==”、“！=”、“<”，“>”、“<=”、“>=”
~~~c++
class person
{
public:
	int m_age;
	bool operator==(person P)
	{
		if(m_age==P.m_age)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}
~~~
-------
### 函数调用运算符重载
+ 函数调用运算符（）也可以重载
+ 重载后使用方式很像函数，因此也叫“仿函数”
+ “仿函数”没有固定写法，很灵活
+ 多用于STL
+ ==只能类内重载==

~~~c++
class person
{
public:
	void operator()(...,...,......)//参数可以不止一个
	{
		...;
	}
}

person(```,```,```);	//使用方法
~~~
-------
## 2.6 继承
### 基础部分
当多个对象有某些共性，同时也有各自的特性，可以考虑使用继承的手法，降低代码量

基本语法
~~~c++
class 子类 : 继承方式 父类
~~~
>子类也成为派生类，父类也称基类

------
### 继承方式
一共有三种
+ 公共继承 public
+ 保护继承 protected
+ 私有继承 private
~~~c++
//父类
class A
{
public:
	int a;
protected:
 	int b;
private:
	int c;
}

//子类
class B : public A		class B : protected A		classB : private A
{						{							{
public:					protected:					private:
	int a;					int a;						int a;
protected:					int b;						int b;
 	int b;				不可访问:					 不可访问:
不可访问:					 int c;						 int c;
	int c;
}						}							}
~~~
>+ 无论是子类的继承方式如何，父类的private都不可访问
>+ 如或是public继承，父类其余类别权限不变
>+ 如果是protected或private继承，父类中所有可继承成员都变成继承方式的访问权限

==父类中的私有成员虽然子类访问不到，但是继承过程中还是都继承到子类中，只是编译器给隐藏了，因此计算子类对象所占的内存空间时，还是要算上父类的私有成员占的空间==

-----
### 继承中的构造和析构顺序
先构造父类，再构造子类；先析构子类，再析构父类

-----
### 继承中同名成员处理
如果想通过子类对象访问到父类中同名成员，需要加上作用域

==子类中的同名成员函数会隐藏父类中的同名成员函数，即使两个同名函数形参不一样，也不能实现函数重载的效果==

~~~c++
son.m_a;	  	 //访问的是子类中的对象
son.base::m_a;	 //访问的是父类中同名的成员属性
son.func();	  	 //访问的是子类中的对象
son.base::func();//访问的是父类中同名的成员函数
~~~

------
### 继承同名静态成员处理方式
处理方式和上一节一样
+ 直接调用，对应着子类中同名成员
+ 加上父类作用域，对应父类中同名成员

同名静态成员属性访问方法如下，==成员函数==的访问方式同理
~~~c++
//通过对象访问
cout<< son.m_a;//子类中的对象
cout<< son.base::m_a;//父类中的对象
//通过类名访问
cout<< person::m_a;//子类中的对象
cout<< person::base::m_a;//父类中的对象
~~~

-----
### 多继承语法
语法
~~~c++
class 子类 : 继承方式 父类1 , 继承方式 父类2 ...
~~~

==不同父类中有同名成员，要加作用域区分==

-----
### 菱形继承
情况如下：存在一个类A，同时被类B和类C继承，此时有一个类D继承了类B和类C，则会出现类D中有两份类A的成员
~~~c++
class A
{
public:
	int m_temp;
};
class B : public A {};
class C : public A {};
class D : public B , public C {};
~~~
上述继承结构下，类D的对象中会存在两个m_temp实体，==占用不同的存储空间==。调用不同实体的语法是使用作用域
~~~c++
D p1;
p1.B::m_temp;//类B继承的
p1.C::m_temp;//类C继承的，可与类B继承的m_temp分别赋不同值
~~~
避免上述菱形继承问题可以使用虚继承
~~~c++
class A
{
public:
	int m_temp;
};
class B : virtual public A {};
class C : virtual public A {};
class D : public B , public C {};
~~~
这样类D中只有一个m_temp实体。尽管依然可以使用作用域去调用“p1.B::m_temp”和“p1.C::m_temp”这两个看似是不同的m_temp，但是在内存空间上，这两个m_temp占用相同的内存空间，类B和类C中为了保证指向相同的内存空间，使用了偏移地址的手法，存储的是指向m_temp所在内存空间地址的偏移量

-----
## 2.7 多态
### 基本概念
多态分为两类
+ 静态多态：函数重载和运算符重载属于静态多态，复用函数名
+ 动态多态：派生类和虚函数实现运行时多态

静态多态和动态多态之间的区别：
+ 静态多态的函数地址早绑定--编译阶段确定函数地址
+ 动态多态的函数地址晚绑定--运行阶段确定函数地址

动态多态满足条件
+ 有继承关系
+ 子类要重写父类中的虚函数（重载和重写不同，重写要求函数名称和形参列表完全相同）
+ 子类重写父类虚函数时，可以加“virtual”关键字，也可以不加

==动态多态使用方法（情形）==

+ 父类的指针/引用指向一个子类的对象，以此调用子类中的成员

-----
### 底层原理
首先，类的成员属性和成员函数在地址上是分开存储的，因此一个类只有成员函数的话，这个类的sizeof只有一个字节；如果这个类加了“virtual”关键字后，这个类的sizeof占4个（根据硬件条件决定）字节，这4个字节是存储虚函数（表）指针

==以下内容是抽象的描述存储空间情况==
~~~c++
class father						class son
{									{
	//虚函数（表）指针						//虚函数（表）指针
	一个指向虚函数表的指针					  一个指向虚函数表的指针
}									}

虚函数表							 虚函数表
{									{
						  				//如果子类中没有重写父类虚函数
	指向父类虚函数的指针					  指向父类虚函数的指针
										//如果子类中重写了父类虚函数
										指向子类虚函数的指针
}									}
~~~

~~~c++
class father
{
public:
	virtual void func()
	{
		...;
	}
};
class son : public father
{
	void func()
	{
		...;
	}
};

void main()
{
	son S;
	father& F1=S;
	father* F2=S;
	F1.func();//调用的是子类对象S中的函数func()
	F2.func();//调用的是子类对象S中的函数func()
}
~~~
结合上述两部分代码可以看出，子类如果继承父类，则==也会继承父类的虚函数表指针和一个虚函数表==。在实际代码运行中，用父类的引用（指针）指向子类时，其实==新的父类对象中的虚函数表指针的值被赋予的就是子类对象中的虚函数表指针==。因此此时使用父类对象时，因为==虚函数表指针已经指向了子类对象的虚函数表==，自然调用的就是子类对象虚函数表中存放的函数指针指向的对象

-----
### 纯虚函数和抽象类
通常在编程中，父类中的虚函数都是不写什么的，只需要等待调用子类中的函数即可，因此可以将虚函数设置为纯虚函数

纯虚函数语法
~~~c++
virtual 返回值类型 函数名 (参数列表) = 0;
~~~
抽象类特点：
+ 当一个类中有了纯虚函数，就是抽象类
+ 无法实例化对象，==但是可以定义对象指针和引用==
+ 子类必须重写父类中的纯虚函数，否则也是抽象类

------
### 虚析构和纯虚析构
~~~c++
class father
{
public:
	virtual void func() = 0;
};
class son ： public father
{
public:
	son (string N)
	{
		name = new string(N)
	}
	~son()
	{
		if(name!=NULL)
		{
			delete name;
			name=NULL;
		}
	}
	
	void func()
	{
		...;
	}
	
	string* name;
};

void main()
{
	father* F = new son("myname")
	delete F;//此时只会调用父类的析构函数，因为消除的是父类指针
}
~~~
上述父类对象析构时，子类在堆区申请的空间没有得到释放，用如下格式解决。采用虚析构和纯虚析构就可以在析构父类时吧子类的一并析构,==并且是按顺序，先析构子类再析构父类==
~~~c++
class father
{
public:
	virtual ~father()//虚析构
	{
		...;
	}
	virtual void func() = 0;
};
~~~
~~~c++
class father
{
public:
	virtual ~father() = 0;//纯虚析构
	virtual void func() = 0;
};
father::~father()
{
	...;
}
~~~
>纯虚析构要另外实现析构函数，可以在类外定义一下，记得加上作用域

# 3 模板与STL

## 模板

C++中除了面向对象的编程思想，还有泛型编程这一思想。这一思想主要是通过模板这一技术实现，提高代码的复用率。

### 函数模板

--------
#### 基本用法
语法
~~~c++
template<typename T1,typename T2,....>
~~~
>+ template---------声明创建模板
>+ typename----------表明其后面的符号是一种数据类型，可用class完成
>+ T--------通用的数据类型，名字可以随便换，但是一般都是写成大写

使用案例
~~~c++
template<typename T>//声明一个模板，告诉编译器后面的T不要报错，是一个通用的数据类型
void swap(T a,T b)
{
	T c;
	c=a;
	b=a;
	a=c;
}

void main()
{
	int a;
	int b;
	//第一种用法：编译器自动类型推导
	swap(a,b);//编译器根据实参类型自动推导T代表的数据类型,不会发生隐式类型转换
	//第二种用法：显示指定类型
	swap<int>(a,b);//可以发生隐式类型转换
}
~~~
>+ 自动推导必须保证推导出的结果是一致的
>+ 写了模板声明后必须要用，不然报错
>+ 使用函数模板的自动推导编译器不会进行隐式类型转换
>+ 显示指定类型的使用模板，编译器可以进行隐式类型转换

-------
#### 普通函数和函数模板的调用规则
+ 1、如果普通模板和普通函数都可以实现，优先调用普通函数，==哪怕普通函数只有声明没有实现==
+ 2、可以通过空模板参数列表来强调调用函数模板
~~~c++
func<>(a,b);
~~~
+ 3、函数模板也可以重载
+ 4、如果函数模板可以产生更好的匹配，优先调用函数模板，==比如虽然调用普通函数可以实现,但是需要进行类型转换,此时编译器就会考虑能否用函数模板实现不类型转换的调用,如果可以就会调用函数模板==

-------
#### 类模板重载问题
类模板重载需要用以下语法
~~~c++
template<> fuc(....)
{
	....;
}
~~~
前面加“template<>”是告诉编译器这是一个模板的重载。如果不加，就是普通函数的重载。这两种情况根据上一节的知识可以知道调用优先级是不一样的

-------
#### 模板与自定义数据类型之间的问题

使用模板时,如果用到了自定义数据类型,除了进行运算符重构来防止模板内代码出错,还可以使用模板的模板来为自定义数据类型单独创建一个模板,例子如下
~~~c++
template <typename T>
bool queal(T &a,T &b)
{
	if(a==b)
	{
		return true;
	}else
	{
		return false;
	}
}
template<> bool queal(person &a,person &b)//这里相当于为person这个自定义数据类型单独创建一个模板,防止函数模板不理解自定义数据类型的"=="如何处理
{
	.....;
}
~~~

-----
### 类模板

----
#### 基本语法
~~~c++
template<typename T1,typename T2,....>
后面紧跟着写一个类定义
~~~
>+ template---------声明创建模板
>+ typename----------表明其后面的符号是一种数据类型，可用class完成
>+ T--------通用的数据类型，名字可以随便换，但是一般都是写成大写

例子
~~~c++
template <typename NameType,typename AgeType>
class person
{
public:
	person(NameType name,AgeType age)
	{
		m_name=name;
		m_age=age;
	}
	NameType m_name;
	AgeType m_age;
};

void main()
{
	person<string,int> P1("张三",20);//注意<>里的参数不能省去
}
~~~

-----
#### 类模板的特点与函数模板的区别
+ 类模板==没有自动类型推导的使用方式==
+ 类模板在模板参数列表中可以有默认参数,==只有类模板可以==

对第二点进行举例说明
~~~c++
template <typename NameType = string,typename AgeType = int>//提供了默认的数据类型
class person
{
	...;
};
~~~

-----
#### 类模板成员函数创建时机
+ 普通类中成员函数是在一开始就创建
+ 类模板中的成员函数在调用的时候才创建

-----
#### 类模板的对象作为函数参数传递的方法
~~~c++
//第一种:指定传入类型
void func(person<int> &P)
{
	...;
}
//第二种:参数模板化
template <typename T3>
void func(person<T3> &P)
{
	...;
}
//第三种:整个类模板化
template <typename T>
void func(T &P)//这里吧一个类person模板化了,利用函数模板的自动类型推导就可以直接传递类对象
{
	...;
}
~~~

--------
#### 类模板与继承
+ 当父类是一个类模板时,子类在继承声明时,要指定出父类中的模板参数
~~~c++
class son : public father<...,...,......>//子类继承时,父类模板参数明确
{
	
};
~~~
+ 如果不指定,则编译器不知道给子类分配多少内存
+ 如果向灵活指定父类中的模板参数需要子类也变成类模板
~~~c++
template <typename T2>
class son : public father<T2>//子类也是一个类模板
{
	
}
~~~

-----
#### 类模板成员函数的类外实现
(1)构造函数类外实现
~~~c++
template <typename T1,typename T2>
class person
{
	person(T1 a,T2 b);
};

template <typename T1,typename T2>
person<T1,T2>::person(T1 a,T2 b)
{
	...;
}
~~~
(2)成员函数类外实现
~~~c++
template <typename T1,typename T2>
void person<T1,T2>::func(T1 a,T2 b)
{
	...;
}
~~~

-----
#### 类模板分文件编写

问题的来源：类模板成员函数创建是在调用阶段，分文件编写时链接不到

解决方法：
+ 1）吧原来包含的头文件尾缀.h改为.cpp，包含源文件
+ 2）将声明和实现写道同一个文件中，该后缀为.hpp（非强制，约定俗成），用的时候直接include ".hpp"就可以

| 详细原理：一般在头文件中声明了一个函数，编译器会去找这个函数的实现，所以一般函数份文件编写可以；但是类模板的成员函数是在调用时实现，所以即使头文件中声明了成员函数，编译器不会去找实现，只是看了眼声明，等之后需要用到实现时就找不到了。那两个解决方法本质都是直接吧源文件用include包含，让编译器能直接看到函数实现。

-----
#### 类模板与友元

类内实现
~~~c++
template<class T1,class T2>
class Persson
{
	//全局函数作为友元 类内实现
	friend void printPerson(Person<T1,T2> P)
	{
		cout<<p.Age<<p.Nam;
	}

	private://全局函数 类外实现
	T1 Age;
	T2 Nam;
}
~~~

类外实现
~~~c++
//之所以写前面是因为友元函数用到了Person类，得先看到
template<class T1,class T2>
class Person;

//之所以写前面是因为需要让编译器先看到友元函数模板，才能在类中声明友元函数
template<class T1,class T2>
void printPerson(Person<T1,T2> P)
{
	cout<<p.Age<<p.Nam;
}

template<class T1,class T2>
class Persson
{
	//全局函数作为友元 类外实现
	//加<>是因为这个全局函数是一个模板，需要告诉编译器
	friend void printPerson<>(Person<T1,T2> P);

	private:
	T1 Age;
	T2 Nam;
}
~~~

-----
## STL（标准模板库）
目的就是为了提高通用性，搞一个可以重复使用的模板类似的代码
+ STL广义上分为：容器、算法、迭代器
+ 容器和算法通过迭代器无缝连接
+ STL几乎所有技术都用到了模板

STL六大组件：==容器、算法、迭代器、仿函数、适配器（配接器）、空间配置器==
+ 仿函数：行为类似函数，可以作为算法的某种策略
+ 适配器：用来修饰容器或者仿函数或迭代器接口的东西
+ 空间配置器：负责空间的配置与管理

注意：
+ 每个容器都有只记得专用迭代器
+ 常用容器种类为双向迭代器、随机访问迭代器

### 容器算法迭代器初识
---------
#### vector存放数据类型
+ 容器：vector
+ 算法：for_each()
+ 迭代器：vector< int >::iterator

vector容器建立
~~~c++
include <vector>

vector<int> P;

P.push_back(10);//向容器中插入数据
P.push_back(20);

//通过迭代器访问容器中的数据
vector<int>::iterator itBegin = P.begin();//指向第一个元素
vector<int>::iterator itEnd = P.end();//指向最后一个元素的下一个位置
~~~

vector遍历方法
~~~c++
include <algorithm>

//第一种
for(vector<int>::iterator it = P.begin();it!=P.end();it++)
{
	cout<< *it <<endl;
}

//第二种
void myPrint(int val)
{
	cout<< val <<endl;
}
for_each(P.begin(),P.end(),myPrint);
~~~

----------
## string容器

string是一个类，这个类封装管理着char*，是一个char*型容器

-------
### string构造函数
+ string  C();	//创建一个空的字符串
+ string  C(const char* s);	//使用字符串s初始化
+ string  C(const string& str);	//使用一个string对象初始化另一个string对象
+ string  C(int n,char c);	//使用n个字符c初始化

--------
### string赋值操作
~~~c++
#include <string>

string str;

str=(const char* s);	//char*型字符串赋值给当前字符串
str=(const string &s);	//将string类型字符串赋值给当前字符串
str=(char c);			//也可以单独赋值字符
str.assign(const char *s);		//赋值字符串
str.assign(const char *s,int n);//将字符串前n个字符赋值给str
str.assign(const string &s);	//赋值字符串
str.assign(int n,char c);		//用n个字符c赋值str
~~~

--------
### string字符串拼接
~~~c++
#include <string>

string str,str2;

//都是拼接到末尾
str+="cdf";
str+='c';
str+=str2;
str.append("sdfs");
str.append("sdfd",2);//将字符串前2两个字符拼接到str末尾
str.apend(str2);
str.append(str2,pos,n);//从str2的第pos个位置开始（从0计数），截取n个字符拼接到str
~~~

--------
### string查找和替换
~~~c++
#include <string>

string str;
int position;
int length;

length = str.size();//返回字符串长度

//默认从0开始，！从左往右！寻找有无字符串"sdf"，没有的话返回值为-1
//返回值是int
position = str.find("sdf");	

//rfind是从右往左查，！但是返回值标号还是从左计数！
//返回值是int
position = str.rfind("sdf");

str.replace(pos,n,"efw");	//从pos位置（从0计数）开始，截取str的n个字符，替换为字符串"efw"
~~~

--------
### string字符串比较
~~~c++
str1.compare(str2);
~~~
返回值分为三种：
+ 两个字符串完全一样，返回0
+ 按照字母表顺序，从左到右，str1与str2之间第一个不同的字符作比较，str1的字符比str2字符排序靠前，返回值>0
+ 按照字母表顺序，从左到右，str1与str2之间第一个不同的字符作比较，str1的字符比str2字符排序靠后，返回值<0

--------
### string字符存取
~~~c++
//i从0开始计数，效果和c语言中数组一样
str[i];
str.at(i);//用这个可以检查越界
~~~

--------
### string插入和删除
~~~c++
str.insert(pos,"cdvuy");	//从pos位置（从0计数）后方插入字符串
str.insert(pos,n,'c');		//从pos位置（从0计数）后方插入n个字符
str.erase(pos,n);			//从pos位置（从0计数）起删除n个
~~~

--------
### string子串
~~~c++
str.substr(pos,n);	//从pos位置（从0计数）起截取n个字符串输出
~~~

### 相关函数
==以下相关函数均需要包含头文件`#include<string>`==

#### 将一个整形转化为字符串
`string to_string(int)`

#### 将字符串转化为10进制整形
`stoi()`

#### 截取原字符串中的一段：

==这是string类对象的成员方法==

~~~C++
string substr(int pos = 0,int n ) const;
~~~
参数1：pos是必填参数

参数2：n是可参数，表示取多少个字符，不填表示截取到末尾

该函数功能为：返回从pos开始的n个字符组成的字符串，原字符串不被改变

--------
## vector容器

和数组非常相似，也成为单端数组

vector可以动态扩展，动态扩展指的是找一块更大的空间，将原来的数据搬过去，在释放原存储空间

常用的迭代器有（支持随机访问）：
+ v.begin()		第一个元素
+ v.end()		最后一个元素的后一个位置
+ v.rbegin()	倒数第一个元素
+ v.rend()		第一个元素的前一个位置

--------
### vector构造函数
~~~c++
vector<int> v1;		//无参构造
vector<int> v2(v1.begin(),v1.end());	//吧v1的这个区间内元素传递给v1
vector<int> v3(n,m);	//将n个值为m的元素填充
vector<vector<int>> v4(k,vector<int>(n,m));//初始化二维数组的办法，看不懂就看上一行
vector<int> v5(v3);		//拷贝构造
vector<int> v6{1,2,3};	//直接给值
~~~

--------
### vector赋值操作
~~~c++
vector<int> vec,v1;

vec = v1;		//拷贝赋值
vec.assign(v1.begin(),v2.end());	//迭代器区间内数据赋值
vec.assign(n,m);	//n个m赋值给vec
~~~

--------
### vector容量和大小
~~~c++
v1.empty();			//返回真假表示是否为空
v1.capacity();		//表示目前空间可以存放多少个元素，不是指有几个元素
v1.size();			//有几个元素
//resize如果设定比原来短了，则多的删除
//resize不是设置容量，是设置大小
v1.resize(n);		//重新元素大小为n，默认用0填充多余位置
v1.resize(n,m);		//重新元素大小为n，用m填充多余位置
~~~

--------
### vector插入和删除
~~~c++
v1.push_back(els);			//在末尾添加元素els
v1.pop_back();				//删除末尾的元素

//后面的pos、begin、end必须是迭代器
v1.insert(pos,m);			//在迭代器pos前方插入元素m
v1.insert(pos,n,m);			//在迭代器pos前方插入n个元素m
v1.erase(pos);				//删除迭代器pos处元素
v1.erase(begin,end);		//删除区间【begin,end）的元素，前闭后开
v1.clear();					//清空
~~~

--------
### vector数据存取
~~~c++
v1[i];			//可以用中括号访问
v1.at(i);		//成员函数访问位置i处元素
v1.front();		//返回第一个元素
v1.back();		//返回最后一个元素
~~~

--------
### vector互换容器
~~~c++
v1.swap(v2);		//将v1和v2中的元素互换
~~~
注意：==交换的时候是连着容量一起交换的==

互换容器可以用来收缩内存，节约空间：
假设一个vector容器一开始的容量为1000，之后用resize缩小大小后，容量不会小，造成浪费，此时使用如下代码可以实现收缩内存
~~~c++
vector<int>(v).swap(v);//其中使用v来初始化一个匿名对象，然后v和匿名对象交换
~~~
用v初始化匿名对象，用v的容量和大小初始化了匿名对象的容量和大小。交换后v容量缩小，匿名对象自动释放原容量占用的内存

--------
### vector预留空间
~~~c++
v1.reserve(n);		//预留n个容量
~~~
注意：==预留n个容量，多余的空间不可访问，没有元素。这一点和resize不一样，因为一个是修改容量，一个是修改大小==

------
## deque容器
==相比vector容器，访问速度慢，但是头端插入元素效率更高==，因此也叫双端数组

原理是这样的：
+ 容器内部实际上不是连续的存储空间，是分成一段段缓冲区存储的
+ 有一个中控器，记录着每段存储空间地址，用中控器索引所有元素

常用的迭代器有（支持随机访问）：
+ d.begin()		第一个元素
+ d.end()		最后一个元素的后一个位置

-------
### deque构造
~~~c++
deque<int> d1;	//	无参构造
deque<int> d2(d1,begin(),d1.end());		//已迭代器范围来构造
deque<int> d3(n,m);			//已n个元素m来构造
deque<int> d4(d3);			//拷贝构造
~~~

--------
### dequer赋值操作
~~~c++
deque<int> d1,d2;

d1 = d2;		//拷贝赋值
d1.assign(d2.begin(),d2.end());	//迭代器区间内数据赋值
d1.assign(n,m);	//n个m赋值给d1
~~~

--------
### deque大小操作
~~~c++
d1.empty();			//返回真假表示是否为空
d1.size();			//有几个元素
//resize如果设定比原来短了，则多的删除
d1.resize(n);		//重新元素大小为n，默认用0填充多余位置
d1.resize(n,m);		//重新元素大小为n，用m填充多余位置
~~~
==deque没有容量这个概念，和vector不同==

--------
### deque插入和删除
~~~c++
d1.push_back(els);			//在末尾添加元素els
d1.pop_back();				//删除末尾的元素
d1.push_front(els);			//在头部添加元素els
d1.pop_front();				//删除头部的元素

//后面的pos、begin、end必须是迭代器
d1.insert(pos,m);					//在位置pos前方插入元素m
d1.insert(pos,n,m);					//在位置pos前方插入n个元素m
d1.insert(pos,d2.begin(),d2.end());	//在位置pos前方插入d2某个区间的数据
d1.erase(pos);						//删除位置pos处元素
d1.erase(begin,end);				//删除区间【begin,end）的元素，前闭后开
d1.clear();							//清空
~~~

--------
### deque数据存取
~~~c++
v1[i];			//可以用中括号访问
v1.at(i);		//成员函数访问位置i处元素
v1.front();		//返回第一个元素
v1.back();		//返回最后一个元素
~~~

-------
### deque排序
~~~c++
#include <algorithm>

sort(d1.begin(),d1.end());	//将迭代器区间内元素从小到大排序
~~~
==vector也可以用这个排序==

-----
## stack容器
一种先进后出结构，栈的结构

最早进去的元素所在位置叫==栈底==，最后进去的元素叫==栈顶==。栈顶物理地址小

==栈不允许有遍历行为==

-----
### stack常用接口
构造：
~~~c++
stack<int> stk;		//默认构造
stack<int> stk(stk2);//拷贝构造
~~~
赋值
~~~c++
stk1 = stk2;
~~~
存取：
~~~c++
stk.push(elem);	//向栈顶添加元素
stk.top();		//返回一个栈顶元素，但是并不移除它
stk.pop();		//移除一个栈顶元素，不返回它
~~~
大小：
~~~c++
stk.empty();		//判断是否为空，真表示为空
stk.size();			//返回栈的大小
~~~

-----
## queue容器
是一种先进先出容器，和队列类似。==队头和队尾才能被访问，因此不能遍历==。==队头只能出，队尾只能进==

------
### queue常用接口
构造：
~~~c++
queue<int> q;	//默认构造
queue<int> q(q2);//拷贝构造
~~~
赋值
~~~c++
p1 = p2;
~~~
存取
~~~c++
q.push(elem);	//向队尾添加元素
q.pop();		//从队头移除第一个元素，不返回元素
q.front();		//返回第一个元素
q.back();		//返回最后一个元素
~~~
大小：
~~~c++
p.empty();		//判断是否为空
p.size();		//返回大小
~~~

-------
## list容器
在存储空间上不连续的结构，和链表类似，通过结点串起所有数据

优点：
+ 可以快速的在任意位置插入或删除元素

缺点;
+ 遍历速度慢
+ 占用空间较大

list特点：
+ 是一个双向循环的链表
+ 迭代器是双向迭代器，只能前移和后移，不能跳跃访问。==且只能用运算符“++”、“--”来移动==
+ ==插入或者删除元素，并不会影响当前迭代器失效==

------
### list构造
~~~c++
list<int> l;		//默认构造
list<int> l(l2);	//拷贝构造
list<int> l(l2.begin(),l2.end());//用迭代器指定拷贝范围
list<int> l(n,m);	//用n个元素m构造
~~~

------
### list赋值和交换
~~~c++
l1 = l2;				//拷贝赋值
l1.assign(n,m);			//用n个元素m赋值
l1.assign(l2.begin(),l2.end());//用迭代器区间赋值

l1.swap(l2);		//交换两个list容器
~~~

-------
### list大小操作
~~~c++
l.size();
l.empty();
l.resize(n);	//重新指定大小，默认用0填充新位置
l.resize(n,m);	//重新指定大小，用元素m填充新位置
~~~

-------
### list插入和删除
~~~c++
l.push_back(elem);		//向尾部插入数据
l.push_front(elem);		//向头部插入数据
l.pop_back();			//删除尾部数据
l.pop_front();			//删除头部数据

//后面pos必须是迭代器
l.insert(pos,elem);		//在位置pos处前一个位置插入数据elem
l.erase(pos);			//删除pos处数据
l.erase(l2.begin,l2.end());//删除迭代器指定区间
l.remove(elem);			//删除所有和elem匹配的值
l.clear();				//清除所有数据
~~~

-------
### list数据存取
~~~c++
l.front();		//返回第一个元素
l.back();		//返回最后一个元素
list<int>::iterator it = l.begin();
*it = ...;		//利用迭代器解引用访问
~~~

------
### list反转和排序
~~~c++
l.reverse();		//反转所有数据，首位对调
l.sort();			//从小到大排序

bool mysort(int v1,int v2)//提供一个回调函数，告诉编译器排序规则
{
	return v1>v2;
}
l.sort(mysort);		//从大到小排序
~~~
==注意：所有不支持随机访问的迭代器的容器都不能使用标准排序算法sort()。这些容器内部会提供相应排序算法==

## set/multiset容器
==插入元素的时候会自动排序==

set和multiset之间的区别在于，==multiset可以存放重复的元素==

==特别注意！！==
+ 如果要往set容器中存放自定义数据，需要重载小于号，并且要类外重载；或者使用仿函数（详见最后一小节）
+ multiset插入相同元素时（自定义数据按照重载的小于号判断，并不是指完全相同），相同元素排序结果按照插入的先后来
+ multiset中存在相同元素时，用find()查找到的元素是所有相同元素中排序靠前的那个
+ 对于自定义的数据类型，用find()查找相同元素不是指找完全相同的元素，重载小于号时用的哪些成员变量，find()时就用哪些成员变量判断是否找到相同元素

### 构造和赋值
~~~c++
set<int> s1;		//默认构造
set<int> s1(s2);	//拷贝构造

s1.inset(elem);		//插入元素elem
s1 = s2;
~~~

### 大小和交换
==不允许重新指定大小==
~~~c++
s.size();
s.empty();
s.swap(s2);
~~~

### 插入和删除
~~~c++
s.insert(elem);
s.clear();			//清除所有元素
s.erase(pos);		//清除迭代器pos指向位置元素
s.erase(pos1,pos2);	//清除两个迭代器规定区间内元素
s.erase(elem);		//清除指定元素elelm
~~~
判断插入数据是否成功
~~~c++
pair<set<int>::iterator,bool> ret = s.insert(elem);//第一个返回值是插入的位置，第二个是是否成功

if(ret.second==trul)
{
	cout << "插入成功"
}
~~~

### 查找和统计
~~~c++
s.find(elem);		//查找元素elem,返回一个指向该元素位置的迭代器。如果不存在，返回s.end()
s.count(elem);		//统计元素elem的个数，对set来说只有0或1，multiset可以大于1
s.begin();	//返回指向容器中第一个元素的双向迭代器
s.end();	//返回指向容器中最后一个元素的后一个位置的双向迭代器
s.rbegin();	//返回指向容器中最后一个元素的双向迭代器
s.rend();	//返回指向容器中第一个元素的前一个位置的双向迭代器
~~~
+ multiset中存在相同元素时，用find()查找到的元素是所有相同元素中排序靠前的那个
+ ==对于自定义的数据类型，用find()查找相同元素不是指找完全相同的元素，重载小于号时用的哪些成员变量，find()时就用哪些成员变量判断是否找到相同元素==

### pair对组创建
成对出现的数据
~~~c++
pair<tupe ,type> p(value1,value2);
pair<tupe ,type> p = make_pair(value1,value2);
~~~

### 改变默认排序规则
默认是从小到大的

利用仿函数来改变，==要在插入前改变规则==
~~~c++
class mycompare
{
public:
	bool operator()(int v1,int v2)
	{
		return v1 > v2
	}
};

set<int,mycompare> s;//这个s的排序就是从大到小
~~~

## pair容器
创建
~~~C++
pair<T1, T2> p1;            //创建一个空的pair对象（使用默认构造），它的两个元素分别是T1和T2类型，采用值初始化。
pair<T1, T2> p1(v1, v2);    //创建一个pair对象，它的两个元素分别是T1和T2类型，其中first成员初始化为v1，second成员初始化为v2。
make_pair(v1, v2);          // 以v1和v2的值创建一个新的pair对象，其元素类型分别是v1和v2的类型。
~~~

访问数据
~~~C++
p1.first;                   // 返回对象p1中名为first的公有数据成员
p1.second;                 // 返回对象p1中名为second的公有数据成员
~~~

## map/multimap容器

简介：
+ 所有元素都是pair
+ pair中第一个元素为key（键值），第二个元素为value（实值）
+ 所有元素都根据元素的键值自动排序

本质：
+ 是关联式容器，底层用二叉树实现

优点：
+ 可以根据key值快速查找

区别：
+ map中不允许有重复key（键值）的元素，multimap允许
+ unordered_map中元素不排序，map中元素排序

### 构造与赋值
~~~c++
map<int,int> m;		//默认构造
map<int,int> m(m2);	//拷贝构造
m = m2;				//赋值
~~~

### 大小和交换
~~~c++
m.size();
m.empty();
m.swap(m2);		//交换m和m2
~~~

### 插入和删除
~~~c++
//插入
m.insert(pair<int,int>(1,10));				//创建匿名对象初插入
m.insert(map<int,int>::value_type(3,30));	//创建匿名对象初插入
m.insert(make_pair(2,20));					//插入函数返回值
m[key] = value;								//插入key,value这个对

m.erase(pos);		//删除迭代器pos处元素
m.erase(beg,end);	//删除迭代器指定范围内元素
m.erase(key);		//删除key这个键值的元素
m.clear();			//清空
~~~

### 查找和统计
~~~c++
m.find(key);		//返回一个指向key元素的迭代器，如果键值为key的元素不存在，则返回m.end()
m.count(key);		//统计键值为key的元素数量，只有multimap存在返回值大于1的可能
~~~

### 排序
默认排序规则是从小到大，利用仿函数可以改变排序规则
~~~c++
class Mycompare
{
public:
	bool operator()(int v1,int v2) const	//有的编译器这里需要加一个const转变为常函数，有的不需要
	{
		return v1>v2;	//改成从大到小排序
	}
};

map<int,int,Mycompare> m;
~~~

## 函数对象（仿函数）

概念：
+ 重载==函数调用操作符==的类，其对象称为函数对象
+ 函数对象使用重载的()时，行为类似函数调用，因此叫仿函数

本质：
+ ==函数对象不是函数，是一个类==

### 函数对象使用
+ 使用起来和普通函数一样
+ 因为是一个类，同时也可以有自己的状态，记录某些数据
+ 函数对象可以作为参数传递

~~~c++
class MyAdd
{
public:
	int operator()(int v1,int v2)
	{
		return v1+v2;
	}
};

MyAdd add;

cout << add(1,20) << endl;	//输出就是30
~~~

拥有自己的状态，可以作为参数传递
~~~c++
class MyAdd
{
public:
	MyAdd()
	{
		this->count = 0;
	}
	
	int operator()(int v1,int v2)
	{
		this->count++;
		return v1 + v2;
	}
	
	int count;
};

int doAdd(MyAdd &AD,int v1,int v2)	//将类MyAdd作为参数参数传递，让别的函数使用指定仿函数
{
	AD(v1,v2);
	cout << "MyAdd仿函数执行次数为:" << AD.count << endl;	//仿函数内部的成员变量可以记录仿函数执行次数
}

//也可以用匿名函数对象来传递参数
doAdd(MyAdd(),10,20);	//这里MyAdd()就是创建了一个匿名的类对象（函数对象）
~~~

### 谓词
概念：
+ 返回值为bool类型的仿函数成为谓词
+ 如果operator()接受一个参数，那么叫做一元谓词
+ 如果operator()接受两个参数，那么叫做二元谓词

~~~c++
class MyCompare
{
public:
	bool operator()(int v1,int v2)
	{
		return v1 > v2;
	}
};

func(MyCompare());	//此处MyCompare()就是作为一个二元谓词被传递了，本质是一个匿名类对象传递
~~~

### 内建的函数对象
既STL提供的一些仿函数

分类：
+ 算数仿函数
+ 关系仿函数
+ 逻辑仿函数

用法：
+ 引入头文件 #include < function >

#### 算数仿函数
功能：
+ 实现四则运算
+ 其中negate是一员运算，其余为二元运算

~~~c++
plus<int> p;
p(1,2);		//是1加2的计算
minus<int> m;
m(3,1);		//是减法运算
multiplies<int> mul;
mul(2,2);	//是乘法运算
divides<int> div;
div(4,2);	//是除法运算
modulus<int> mod;
mod(6,4);	//取余数,结果为2
negate<int> neg;
neg(5);		//取反，结果为-5
~~~

#### 关系仿函数
~~~c++
equal_to<int> e;		//等于
not_equal_to<int> ne;	//不等于
greater<int> g;			//大于
greater_equal<int> ge;	//大于等于
less<int> l;			//小于
less_equal<int> le;		//小于等于
~~~

#### 逻辑仿函数
~~~c++
logical_and<bool> and;	//与
logical_or<bool> or;	//或
logical_not<bool> not;	//非
~~~

## 知识点
### 迭代器
可以使用如下句子将索引转换为迭代器
~~~C++
auto pos = s.begin() + index;
~~~

# 4 文件操作
提供了一个叫文件流的类，需要包含头文件==<fstream>==

文件类型分为两类
+ 文本文件
+ 二进制文件

文件操作分类三类
+ ofstream:写操作
+ ifstream:读操作
+ fstream:读写操作

------
## 文本文件

### 写文件
+ 1、包含头文件<fstream>
+ 2、创建流对象  std::ofstream ofs;
+ 3、打开文件  ofs.open("文件路径",打开方式);
+ 4、写数据   ofs<<"写入的数据";
+ 5、关闭文件 ofs.close();

文件打开方式

|打开方式|解释|
|-------|--------|
|std::ios::in|为读文件而打开文件|
|std::ios::out|为写文件而打开文件|
|std::ios::ate|初始位置：文件末尾|
|std::ios::app|追加方式写文件，直接从末尾开始写|
|std::ios::trunc|如果文件存在，先删除，再创建|
|std::ios::binary|二进制形式|

注意：
+ ==打开方式可以通过与运算“|”配合使用==，例如std::ios::binary | std::ios::out
+ 利用“<<”写入，”endl“可以当作换行符

### 读文件
+ 1、包含头文件<fstream>
+ 2、创建流对象  std::ifstream ifs;
+ 3、打开文件,上一节提到过  ifs.open("文件路径",打开方式);
+ 4、读数据  四种方式
+ 5、关闭文件 ifs.close();

~~~c++
using namespace std;//注意这里，不然后续都要写上std::

//第一种
char buf[1024] = {0};
while(ifs>>buf)//读完数据会输出一个假，自然停止
{
	cout<<buf<<endl;//注意，这种方法一次读一行，并且不会读取换行符
}

//第二种
char char buf[1024] = {0};
while(ifs.getline(buf,sizeof(buf)))//一次读一行，读完了返回假
{
	cout<<buf<<endl;//注意不会读取换行符
}

//第三种
string buf
while(getline(ifs,buf))//读给string类对象，一次读一行
{
	cout<<buf<<endl;//注意不会读取换行符
}

//第四种
char c;
while( ( c=ifs.get() )!=EOF )//一次读取文件中一个字符，读到结束会返回“EOF”字符
[
	cout<<c;//注意这个方法会读取换行符
]
~~~

注意：
+ 可以通过==“ifs.is_open()”==来判断是否成功打开文件，为真是打开

## 二进制文件
以二进制的方式读写文件

### 写文件
函数原型
~~~c++
ostream& write(const char * buffer,int len);
~~~
示例代码
~~~c++
using namespace std;
class person
{
public:
	char m_name[64];//这里使用char的原因是：因为涉及底层文件操作，用string可能会出现错误，用char稳妥
	int m_age;
};

void main()
{
	ofstream ofs;
	ofs.open("person.txt",ios::out|ios::binary);//这里是用open写，也可以吧这部分直接在创建类对象时初始化，有对应的构造函数
	person P={"name",18};
	ofs.write( (const char *)&P , sizeof(person) );//这里类型强制转化是必须的，因为函数原型需要的就是这种类型
	ofs.close();
}
~~~

### 读文件
函数原型
~~~c++
istream& read(char *buffer,int len);//第一个参数注意是char*型
~~~
代码示例
~~~c++
using namespace std;
class person
{
public:
	char m_name[64];//这里使用char的原因是：因为涉及底层文件操作，用string可能会出现错误，用char稳妥
	int m_age;
};

void main()
{
	ifstream ifs;
	ifs.open("person.txt",ios::in|ios::binary);
	if(ifs.isopen())
	{
		cout<<"打开成功"<<endl;
	}
	person P;
	ofs.read( (char *)&P , sizeof(person) );//这里类型强制转化是必须的，因为函数原型需要的就是这种类型
	
	cout<<P.m_name<<endl<<P.m_age<<endl;//验证一下是否读入正确
	
	ofs.close();
}
~~~

# 5 内存分区模型

## 模型4区域

+ 代码区：存放函数体，由操作系统管理
+ 全局区：存放全局变量、静态变量、常量
+ 栈区：编译器分配，存放函数参数值和局部变量
+ 堆区：由程序员分配，程序结束由操作系统自动回收
### 代码区
代码区的数据是共享的、只读的。共享意味着只用存一份程序在内存中供反复调用
### 堆区
利用new关键字将变量创建在堆区

## new关键字

new的右边是变量，==左边是这个变量对应的指针==

创建一个int数据10，返回其指针
```c++
int *p = new int(10);
```
>此时*p输出为10

创建一个长度为10的数组
```c++
int *p = new int[10];
```
## delete关键字
释放一个int数据
```c++
delete p;
```
释放一个数组
```c++
delete[] p;
```
