#pragma once
#ifndef USEFUL_FUNC
#define USEFUL_FUNC

//general headers
#include <iostream>
#include <string>
#include <iomanip>
#include <stdio.h>

char ReadLastCharOfLine()
{
	int newChar = 0;
	int lastChar;
	fflush(stdout);
	do
	{
		lastChar = newChar;
		newChar = getchar();
	} while ((newChar != '\n') && (newChar != EOF));
	return (char)lastChar;
}

#endif // !USEFUL_FUNC
