/*
 * @file	: licence_checker.hpp
 * @date	: 2022.05.16, "Nate" (cofounder@navifra.com)"
 * @author	:"Nate" (cofounder@navifra.com)"
 * @brief	:  navigation licence checker
 * @remark	:
 * @warning	:
 * 	Copyright(C) "Nate" (cofounder@navifra.com)" NAVIFRA.
 * 	All Rights are Reserved.
 */

#ifndef NAVIFRA_LICENCE_NAVI_H_
#define NAVIFRA_LICENCE_NAVI_H_

using namespace std;

#include "util/logger.hpp"

#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/rsa.h>
#include <openssl/ssl.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
class License {
public:
    License();
    ~License(){};

    std::vector<string> split(string input, char delimiter);

    int padding_;

    RSA* createRSA(unsigned char* key, int pub);

    /* 개인키로 복호화 */
    int private_decrypt(unsigned char* enc_data, int data_len, unsigned char* key, unsigned char* decrypted);

    int CheckLicence();
};
#endif
