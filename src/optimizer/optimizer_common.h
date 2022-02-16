/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#ifndef MANHATTAN_OPTIMIZER_COMMON_H
#define MANHATTAN_OPTIMIZER_COMMON_H

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the MATH_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// CHOLMOD_SOLVER_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.


#ifdef WIN32
// Disable a warning message about dll
// this is a temporary solution
// http://support.microsoft.com/default.aspx?scid=kb;EN-US;168958
#   pragma warning( disable : 4251 )
#endif

// Win 32 DLL export macros
#ifdef WIN32
# ifdef OPTIMIZER_EXPORTS
#   define OPTIMIZER_API  __declspec(dllexport)
# else
#   define OPTIMIZER_API  __declspec(dllimport)
# endif
# else
#define	   OPTIMIZER_API
#endif // WIN32


#endif  // MANHATTAN_OPTIMIZER_COMMON_H
