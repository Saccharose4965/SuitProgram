from __future__ import annotations

try:
    from PySide6 import QtCore, QtGui, QtWidgets

    PYSIDE_AVAILABLE = True
    IMPORT_ERROR: Exception | None = None
except Exception as exc:  # pragma: no cover - import guard for local setup
    QtCore = None
    QtGui = None
    QtWidgets = None
    PYSIDE_AVAILABLE = False
    IMPORT_ERROR = exc

if PYSIDE_AVAILABLE:
    try:
        from PySide6 import QtOpenGL, QtOpenGLWidgets

        QT_OPENGL_AVAILABLE = True
        OPENGL_IMPORT_ERROR: Exception | None = None
    except Exception as exc:  # pragma: no cover - optional OpenGL support
        QtOpenGL = None
        QtOpenGLWidgets = None
        QT_OPENGL_AVAILABLE = False
        OPENGL_IMPORT_ERROR = exc

    try:
        from PySide6 import QtMultimedia

        QT_MULTIMEDIA_AVAILABLE = True
        MULTIMEDIA_IMPORT_ERROR: Exception | None = None
    except Exception as exc:  # pragma: no cover - optional multimedia support
        QtMultimedia = None
        QT_MULTIMEDIA_AVAILABLE = False
        MULTIMEDIA_IMPORT_ERROR = exc
else:
    QtOpenGL = None
    QtOpenGLWidgets = None
    QT_OPENGL_AVAILABLE = False
    OPENGL_IMPORT_ERROR = IMPORT_ERROR
    QtMultimedia = None
    QT_MULTIMEDIA_AVAILABLE = False
    MULTIMEDIA_IMPORT_ERROR = IMPORT_ERROR
