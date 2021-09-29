/*
    Shout outs to this awesome answer on StackOverflow:
     -> https://stackoverflow.com/questions/11886262/reading-public-private-key-from-memory-with-openssl
*/

#pragma once

// Std
#include <memory>
// OpenSSL
#include <openssl/evp.h>
#include <openssl/pem.h>


struct EVP_PKEY_CTX_Deleter {

    void operator()(EVP_PKEY_CTX* context) const {
        if (context) {
            EVP_PKEY_CTX_free(context);
        }
    }

};

struct EVP_PKEY_Deleter {

    void operator()(EVP_PKEY* key) const {
        if (key) {
            EVP_PKEY_free(key);
        }
    }

};


struct EVP_CIPHER_CTX_Deleter {

    void operator()(EVP_CIPHER_CTX* context) const {
        if (context) {
            EVP_CIPHER_CTX_free(context);
        }
    }

};

struct BIO_Deleter {

    void operator()(BIO* bio) const {
        if (bio) {
            BIO_free(bio);
        }
    }

};


typedef std::unique_ptr<EVP_PKEY_CTX, EVP_PKEY_CTX_Deleter> evp_pkey_ctx_ptr;
typedef std::unique_ptr<EVP_PKEY, EVP_PKEY_Deleter> evp_pkey_ptr;

typedef std::unique_ptr<EVP_CIPHER_CTX, EVP_CIPHER_CTX_Deleter> evp_cipher_ctx_ptr;

typedef std::unique_ptr<BIO, BIO_Deleter> bio_ptr;
