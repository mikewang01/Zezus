/***************************************************************
 * Name:      __VERSION_H__
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */

#ifndef __VERSION_H__
#define __VERSION_H__

/* Exported constants --------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/



#define VER_MAJOR 0
#define VER_MINOR 1
#define REVISION  0
#define BUILD_NUM 0

#define  GET_VERSION() ((VER_MAJOR<<24) | VER_MINOR<<16 | REVISION<<8 | BUILD_NUM)





#endif // __VERSION_H__

/*

应根据下面的约定使用这些部分：

Major ：具有相同名称但不同主版本号的程序集不可互换。例如，这适用于对产品的大量重写，这些重写使得无法实现向后兼容性。

Minor ：如果两个程序集的名称和主版本号相同，而次版本号不同，这指示显著增强，但照顾到了向后兼容性。例如，这适用于产品的修正版或完全向后兼容的新版本。

Build ：内部版本号的不同表示对相同源所作的重新编译。这适合于更改处理器、平台或编译器的情况。

Revision ：名称、主版本号和次版本号都相同但修订号不同的程序集应是完全可互换的。这适用于修复以前发布的程序集中的安全漏洞。

程序集的只有内部版本号或修订号不同的后续版本被认为是先前版本的修补程序 (Hotfix) 更新。

版本号管理策略

一、GNU 风格的版本号管理策略：

1．项目初版本时，版本号可以为 0.1 或 0.1.0, 也可以为 1.0 或 1.0.0，如果你为人很低调，我想你会选择那个主版本号为 0 的方式；
2．当项目在进行了局部修改或 bug 修正时，主版本号和子版本号都不变，修正版本号加 1；
3. 当项目在原有的基础上增加了部分功能时，主版本号不变，子版本号加 1，修正版本号复位为 0，因而可以被忽略掉；
4．当项目在进行了重大修改或局部修正累积较多，而导致项目整体发生全局变化时，主版本号加 1；
5．另外，编译版本号一般是编译器在编译过程中自动生成的，我们只定义其格式，并不进行人为控制。
*/
