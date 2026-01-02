/* Dummy C extension to force setuptools to run build_ext */
#include <Python.h>

static PyModuleDef _dummy_module = {
    PyModuleDef_HEAD_INIT,
    "_dummy",
    NULL,
    -1,
    NULL
};

PyMODINIT_FUNC PyInit__dummy(void) {
    return PyModule_Create(&_dummy_module);
}









