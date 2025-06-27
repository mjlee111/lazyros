# How to release to pypi

This document shows how to release to pypi. Please use the following procedure to register a package on pypi.

1. Install [poetry](https://python-poetry.org/) (Only required for first time)
2. Make a account of [pypi](https://pypi.org/) and get API token (Only required for first time)
3. Register your pypi account to poetry (Only required for first time)
4. Build package
5. Publish package to pypi.



## 1. Install poetry (Only required for first time)

```
pip install poetry
```



## 2. Make a account of pypi and get API token (Only required for first time)

1. Go to [pypi](https://pypi.org/)
2. Click right upper *Register* button to make user account
3. Follow account registration procedure
4. After sign in pypi, Get a API token
   1. Sign in pypi
   2. Click Account config
   3. Make new API token



## 3. Register your pypi account to poetry (Only required for first time)

```
poetry config pypi-token.pypi "pypi-API-token"
```

For example.

```
poetry config pypi-token.pypi pypi-UgAIcHlwa....
```



## 4. Build package

```
poetry build
```



## 5. Publish package to pypi

```
poetry publish
```

