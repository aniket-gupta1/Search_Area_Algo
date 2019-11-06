
class Graph:
    def __init__(self, nVertices):
        self.nVertices = nVertices
        self.adjmatrix = [[0 for i in range(nVertices)] for j in range(nVertices)]
    
    def __str__(self):
        return str(self.adjmatrix)
    
    def addEdge(self, v1, v2):
        self.adjmatrix[v1][v2] = 1
        self.adjmatrix[v2][v1] = 1

        
    def removeEdge(self, v1, v2):
        if self.adjmatrix[v1][v2] == 1:
            self.adjmatrix[v1][v2] = 0
            self.adjmatrix[v2][v1] = 0
        return 
        
    def containsEdge(self, v1, v2):
        return bool(self.adjmatrix[v1][v2])
    
    def DFS(self):
        visited = {i:False for i in range(self.nVertices)}
        for i in range(self.nVertices):
            if not visited[i]:
                self.__DFShelper(i, visited)
        
    def __DFShelper(self, sv, visited):
        print(sv)
        visited[sv] = True
        for i in range(self.nVertices):
            if self.adjmatrix[sv][i] > 0 and visited[i] == False:
                self.__DFShelper(i, visited)
    
    def BFShelper(self, sv, visited):
        import queue
        q = queue.Queue()
        q.put(sv)
        q.put(None)
        while not q.empty():
            x = q.get()
            if x != None:
                print(x, end= " ")
                visited[x] = True
                for i in range(self.nVertices):
                    if self.adjmatrix[x][i] > 0 and visited[i] is False:
                        q.put(i)
                        visited[i] = True
                    if i == self.nVertices - 1:
                        q.put(None)
            else:
                print()
    
    def BFS(self):
        visited = {i: False for i in range(self.nVertices)}
        for i in range(self.nVertices):
            if not visited[i]:
                self.BFShelper(i, visited)
            
    def pathBFShelper(self, s, e, visited):
        import queue
        q = queue.Queue()
        q.put(s)
        parent_dict = dict()
        l = []
        while not q.empty():
            x = q.get()
            visited[x] = True
            for i in range(self.nVertices):
                if not visited[i] and self.adjmatrix[x][i] == 1:
                    visited[i] = True
                    parent_dict[i] = x
                    q.put(i)
                    if i == e:
                        cursor = i
                        l.append(cursor)
                        while parent_dict[cursor] != s:
                            l.append(parent_dict[cursor])
                            cursor = parent_dict[cursor]
                        l.append(s)
                        return l
        
        return l
                                

    def pathBFS(self, i, j):
        visited = {k:False for k in range(self.nVertices)}
        return self.pathBFShelper(i, j, visited)