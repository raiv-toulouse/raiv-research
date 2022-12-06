
from PyQt5 import QtGui,QtWidgets
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT)
# Matplotlib Figure object
from matplotlib.figure import Figure
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class MplCanvas(FigureCanvas):
     """Class to represent the FigureCanvas widget"""
     def __init__(self):
         # setup Matplotlib Figure and Axis
         self.fig = Figure()
         self.ax = self.fig.add_subplot(111)
         # initialization of the canvas
         FigureCanvas.__init__(self, self.fig)
         # we define the widget as expandable
         FigureCanvas.setSizePolicy(self,
         QtWidgets.QSizePolicy.Expanding,
         QtWidgets.QSizePolicy.Expanding)
         # notify the system of updated policy
         FigureCanvas.updateGeometry(self)
         
class NavigationToolbar(NavigationToolbar2QT):
    # only display the buttons we need
#    toolitems = [t for t in NavigationToolbar2QT.toolitems if
#                 t[0] in ('Home', 'Pan', 'Zoom', 'Save')]
    def __init__(self, plotCanvas,parent):
        # create the default toolbar
        NavigationToolbar2QT.__init__(self,plotCanvas,parent)
        self.canvas = plotCanvas
        # add new toolbar buttons 
        self.addAction(self._icon('filesave.png'),'Export to CSV file',self.saveCSV)
        
    def saveCSV(self):
        ax=self.canvas.fig.gca()   
        if ax.lines!=[]:  # Il y a au moins une courbe
            dicoSeries = {}
            dicoSeries['X'] = pd.Series(ax.lines[0].get_xdata())
            for i in range(len(ax.lines)-2):  # -2 pour ne pas exporter les courbes dues au réticule
                dicoSeries['Y'+str(i)] = pd.Series(ax.lines[i].get_ydata())
            df = pd.DataFrame(dicoSeries)
            df.to_csv('monFichier.csv',index=False)

         
class MplWidgetAvecToolbar(QtWidgets.QWidget):
     """Widget defined in Qt Designer"""
     def __init__(self, avecToolBar = True, parent = None):
         # initialization of Qt MainWindow widget
         QtWidgets.QWidget.__init__(self, parent)
         # set the canvas to the Matplotlib widget
         self.canvas = MplCanvas()
         self.mpl_toolbar = NavigationToolbar(self.canvas, self)
         # create a vertical box layout
         self.vbl = QtWidgets.QVBoxLayout()
         # add mpl widget to vertical box
         self.vbl.addWidget(self.canvas)
         if avecToolBar:
             # Ajout de la toolbar
             self.vbl.addWidget(self.mpl_toolbar)
         # set the layout to th vertical box
         self.setLayout(self.vbl)


class MplWidget(QtWidgets.QWidget):
    """Widget defined in Qt Designer"""

    def __init__(self, parent=None):
        # initialization of Qt MainWindow widget
        MplWidgetAvecToolbar.__init__(self, False, parent)