AC_REVISION($Revision: 1.14 $)
dnl
dnl Indicate the package name and the version to automake
dnl
AC_INIT([libserial], [0.6.0rc3])
AC_CONFIG_SRCDIR([src/SerialStream.cc])
AM_CONFIG_HEADER(config.h)
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE
AM_MAINTAINER_MODE

dnl Checks for programs.
AC_PROG_CXX
AC_PROG_AWK
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
AC_PROG_LIBTOOL

AC_CHECK_PROG(DOCBOOK2PDF,docbook2pdf,docbook2pdf,no)
if test "x$DOCBOOK2PDF" == "xno"; then
      AC_MSG_WARN([Could not find docbook2pdf. Disabling creation of user manual therefore.])
      HAVE_DOCBOOK2PDF="no"
else
      HAVE_DOCBOOK2PDF="yes"
fi
AC_SUBST(DOCBOOK2PDF)
AM_CONDITIONAL(HAVE_DOCBOOK2PDF, test x$HAVE_DOCBOOK2PDF = xyes)

dnl Checks for header files.
AC_CHECK_HEADERS(fcntl.h unistd.h)

dnl Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_C_INLINE

AC_ARG_WITH([python],
	AS_HELP_STRING([--without-python], [Disable Python bindings]),
	[], [with_python=yes])
AM_CONDITIONAL([PYTHON], [test "${with_python}" != "no"])

AC_OUTPUT([Makefile
doxygen.conf
libserial.spec
doc/UML/Makefile
doc/Makefile
examples/Makefile
src/Makefile
sip/configure.py
sip/Makefile
libserial.pc])
