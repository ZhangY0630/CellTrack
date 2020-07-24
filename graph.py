from cell import Cell
import matplotlib.pyplot as plt
import numpy as np
class Graph():
    def __init__(self,mask,height,width):
        super().__init__()
        self.cells = {}
        self.mask = mask
        self.height = height
        self.width = width

        for  i in range(width):
            for j in range(height):
                if mask[i][j] == 0:
                    continue
                self.addCell(mask[i][j],i,j)

    def checkCellExist(self,id):
        for key in self.cells:
            if key == id:
                return True
            
        return False

    def addCell(self,id,x,y):
        if not self.checkCellExist(id):
            empty = np.zeros((self.width,self.height))
            newCell = Cell(id,empty,self.mask)
            self.cells[id] = newCell
        
        self.cells[id].addPixel(x,y)


    def get_cell(self,id):
        return self.cells[id]

    def cell_size(self):
        return len(self.cells)

    def showM(self):
        plt.imshow(self.mask)
        plt.show()