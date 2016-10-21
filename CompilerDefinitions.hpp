//SprintCompilerDefinitions.hpp
/*
@author: Bryan Wodi <talk2kamp@gmail.com>
@date:	Aug 13, 2016
*/

#ifndef _SPRINT_COMPILER_DEFINITIONS_H_
#define _SPRINT_COMPILER_DEFINITIONS_H_

#ifndef __FILENAME__
#define __FILENAME__ __FILE__
#endif

#ifdef DEBUG
#define VERBOSEHEADER "[VERBOSE] " << __FILENAME__ << ": " << __LINE__  << ") "
#define VERBOSE(x) std::cout<< VERBOSEHEADER << x <<std::endl;
#define VERBOSETP(x,y) std::cout << VERBOSEHEADER << x << y << std::endl;
#define VERBOSENEL(x) std::cout<< VERBOSEHEADER << x
#else
#define VERBOSEHEADER
#define VERBOSE(x)
#define VERBOSETP(x,y)
#define VERBOSENEL(x)
#endif

#ifndef ERROR
#define ERRORHEADER "[ERROR]" << __FILENAME__ <<": " << __LINE__ << ") "
#define ERROR(x) std::cout << ERRORHEADER << x << std::endl;
#endif

#define _USE_MATH_DEFINES

#endif