# -*- coding: utf-8 -*-
"""
Created on Mon Apr 29 09:05:26 2019

@author: kirthi
"""


DEBUG = True

#%% - Common Functions written here
def print_debug(*objects):
    if DEBUG == True:
        print(*objects)
