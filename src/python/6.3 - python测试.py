#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys,numpy


class MoveItIkDemo:
    def __init__(self):
        # size=(29,43)
        # a = ['*']*43
        
        # for i in range(29):
        #     b[i]=a
        N = [[0]*43 for i in range(29)]
        for i in range(29):
            print(N[i])
        print(N[1][1])


if __name__ == "__main__":
    MoveItIkDemo()


    
    
