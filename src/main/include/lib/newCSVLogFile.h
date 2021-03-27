#include <fstream>
#include <string>
#include <cstdarg>
#pragma once

class newCSVLogFile
{
public:
    newCSVLogFile(char *filename, char *head);
    ~newCSVLogFile();
    void log(double value[3], int arraySize);

private:
    FILE *m_file;
};