/**
 * Copyright (c) 2011-2018 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include "../../../Repetier.h"
#include <math.h>
#include "FatFile.h"
#include "FmtNumber.h"
#if SDSUPPORT
//------------------------------------------------------------------------------
// print uint8_t with width 2
static void print2u(print_t* pr, uint8_t v) {
  char c0 = '?';
  char c1 = '?';
  if (v < 100) {
    c1 = v/10;
    c0 = v - 10*c1 + '0';
    c1 += '0';
  }
  pr->write(c1);
  pr->write(c0);
}
//------------------------------------------------------------------------------
static void printU32(print_t* pr, uint32_t v) {
  char buf[11];
  char* ptr = buf + sizeof(buf);
  *--ptr = 0;
  pr->write(fmtDec(v, ptr));
}
//------------------------------------------------------------------------------
static void printHex(print_t* pr, uint8_t w, uint16_t h) {
  char buf[5];
  char* ptr = buf + sizeof(buf);
  *--ptr = 0;
  for (uint8_t i = 0; i < w; i++) {
    char c = h & 0XF;
    *--ptr = c < 10 ? c + '0' : c + 'A' - 10;
    h >>= 4;
  }
  pr->write(ptr);
}
//------------------------------------------------------------------------------
void FatFile::dmpFile(print_t* pr, uint32_t pos, size_t n) {
  char text[17];
  text[16] = 0;
  if (n >= 0XFFF0) {
    n = 0XFFF0;
  }
  if (!seekSet(pos)) {
    return;
  }
  for (size_t i = 0; i <= n; i++) {
    if ((i & 15) == 0) {
      if (i) {
        pr->write(' ');
        pr->write(text);
        if (i == n) {
          break;
        }
      }
      pr->write('\r');
      pr->write('\n');
      if (i >= n) {
        break;
      }
      printHex(pr, 4, i);
      pr->write(' ');
    }
    int16_t h = read();
    if (h < 0) {
      break;
    }
    pr->write(' ');
    printHex(pr, 2, h);
    text[i&15] = ' ' <= h && h < 0X7F ? h : '.';
  }
  pr->write('\r');
  pr->write('\n');
}
//------------------------------------------------------------------------------
//bool FatFile::ls(print_t* pr, uint8_t flags, uint8_t indent) {
//  FatFile file;
//  if (!isDir() || getError()) {
//    DBG_FAIL_MACRO;
//    goto fail;
//  }
//  rewind();
//  while (file.openNext(this, O_RDONLY)) {
//    if (!file.isHidden() || (flags & LS_A)) {
//    // indent for dir level
//      for (uint8_t i = 0; i < indent; i++) {
//        pr->write(' ');
//      }
//      if (flags & LS_DATE) {
//        file.printModifyDateTime(pr);
//        pr->write(' ');
//      }
//      if (flags & LS_SIZE) {
//        file.printFileSize(pr);
//        pr->write(' ');
//      }
//      file.printName(pr);
//      if (file.isDir()) {
//        pr->write('/');
//      }
//      pr->write('\r');
//      pr->write('\n');
//      if ((flags & LS_R) && file.isDir()) {
//        if (!file.ls(pr, flags, indent + 2)) {
//          DBG_FAIL_MACRO;
//          goto fail;
//        }
//      }
//    }
//    file.close();
//  }
//  if (getError()) {
//    DBG_FAIL_MACRO;
//    goto fail;
//  }
//  return true;
//
// fail:
//  return false;
//}
//------------------------------------------------------------------------------
bool FatFile::printCreateDateTime(print_t* pr) {
  dir_t dir;
  if (!dirEntry(&dir)) {
    DBG_FAIL_MACRO;
    goto fail;
  }
  printFatDate(pr, dir.creationDate);
  pr->write(' ');
  printFatTime(pr, dir.creationTime);
  return true;

fail:
  return false;
}
//------------------------------------------------------------------------------
void FatFile::printFatDate(print_t* pr, uint16_t fatDate) {
  printU32(pr, FAT_YEAR(fatDate));
  pr->write('-');
  print2u(pr, FAT_MONTH(fatDate));
  pr->write('-');
  print2u(pr, FAT_DAY(fatDate));
}
//------------------------------------------------------------------------------
void FatFile::printFatTime(print_t* pr, uint16_t fatTime) {
  print2u(pr, FAT_HOUR(fatTime));
  pr->write(':');
  print2u(pr, FAT_MINUTE(fatTime));
  pr->write(':');
  print2u(pr, FAT_SECOND(fatTime));
}
//------------------------------------------------------------------------------
/** Template for FatFile::printField() */
template <typename Type>
static int printFieldT(FatFile* file, char sign, Type value, char term) {
  char buf[3*sizeof(Type) + 3];
  char* str = &buf[sizeof(buf)];

  if (term) {
    *--str = term;
    if (term == '\n') {
      *--str = '\r';
    }
  }
#ifdef OLD_FMT
  do {
    Type m = value;
    value /= 10;
    *--str = '0' + m - 10*value;
  } while (value);
#else  // OLD_FMT
  str = fmtDec(value, str);
#endif  // OLD_FMT
  if (sign) {
    *--str = sign;
  }
  return file->write(str, &buf[sizeof(buf)] - str);
}
//------------------------------------------------------------------------------

int FatFile::printField(float value, char term, uint8_t prec) {
  char buf[24];
  char* str = &buf[sizeof(buf)];
  if (term) {
    *--str = term;
    if (term == '\n') {
      *--str = '\r';
    }
  }
  str = fmtFloat(value, str, prec);
  return write(str, buf + sizeof(buf) - str);
}
//------------------------------------------------------------------------------
int FatFile::printField(uint16_t value, char term) {
  return printFieldT(this, 0, value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(int16_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint16_t)value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(uint32_t value, char term) {
  return printFieldT(this, 0, value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(int32_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint32_t)value, term);
}
//------------------------------------------------------------------------------
bool FatFile::printModifyDateTime(print_t* pr) {
  dir_t dir;
  if (!dirEntry(&dir)) {
    DBG_FAIL_MACRO;
    goto fail;
  }
  printFatDate(pr, dir.lastWriteDate);
  pr->write(' ');
  printFatTime(pr, dir.lastWriteTime);
  return true;

fail:
  return false;
}
//------------------------------------------------------------------------------
size_t FatFile::printFileSize(print_t* pr) {
  char buf[11];
  char *ptr = buf + sizeof(buf);
  *--ptr = 0;
  ptr = fmtDec(fileSize(), ptr);
  while (ptr > buf) {
    *--ptr = ' ';
  }
  return pr->write(buf);
}

//########################################################################################
//#################### Repetier Erstatzfunktionen fuer ls mit pr->xxx ####################
//########################################################################################

void FatFile::lsRecursive(uint8_t level)
{
	FatFile file;

	rewind();

	while (file.openNext(this, O_READ)) {
		file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
		HAL::pingWatchdog();
		if (file.isHidden()) {
			file.close();
			continue;
		}
		// if (! (file.isFile() || file.isDir())) continue;
		if (strcmp(tempLongFilename, "..") == 0) {
			file.close();
			continue;
		}
		if (tempLongFilename[0] == '.') {
			file.close();
			continue; // MAC CRAP
		}
		if (file.isDir()) {
			if (level >= SD_MAX_FOLDER_DEPTH) {
				file.close();
				continue; // can't go deeper
			}
			if (level) {
				Com::print(fullName);
				Com::printF(Com::tSlash);
			}

			Com::print(tempLongFilename);
			Com::printFLN(Com::tSlash); // End with / to mark it as directory entry, so we can see empty directories.

			char *tmp;
			// Add directory name
			if (level) strcat(fullName, "/");
			strcat(fullName, tempLongFilename);
			file.lsRecursive(level + 1);
			
			// remove added directory name
			if ((tmp = strrchr(fullName, '/')) != NULL)
				*tmp = 0;
			else
				*fullName = 0;
		}
		else { // is filename
			if (level) {
				Com::print(fullName);
				Com::printF(Com::tSlash);
			}
			
			Com::print(tempLongFilename);
#if SD_EXTENDED_DIR
			Com::printF(Com::tSpace, (long)file.fileSize());
#endif
			Com::println();
		}
		file.close();
	}
}

/** List directory contents.
*
* \param[in] pr Print stream for list.
*
* \param[in] flags The inclusive OR of
*
* LS_DATE - %Print file modification date
*
* LS_SIZE - %Print file size.
*
* LS_R - Recursive list of subdirectories.
*
* \param[in] indent Amount of space before file name. Used for recursive
* list to indicate subdirectory level.
*/
void FatFile::ls() {
	*fullName = 0;
	lsRecursive(0);
}
#endif
