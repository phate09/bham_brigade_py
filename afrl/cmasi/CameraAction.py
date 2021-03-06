#! /usr/bin/python

import sys, struct
import xml.dom.minidom
from lmcp import LMCPObject

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

from afrl.cmasi import PayloadAction


class CameraAction(PayloadAction.PayloadAction):

    def __init__(self):
        PayloadAction.PayloadAction.__init__(self)
        self.LMCP_TYPE = 18
        self.SERIES_NAME = "CMASI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.CameraAction"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 4849604199710720000
        self.SERIES_VERSION = 3

        #Define message fields
        self.HorizontalFieldOfView = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadAction.PayloadAction.pack(self))
        buffer.extend(struct.pack(">f", self.HorizontalFieldOfView))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadAction.PayloadAction.unpack(self, buffer, _pos)
        self.HorizontalFieldOfView = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadAction.PayloadAction.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "HorizontalFieldOfView" and len(e.childNodes) > 0 :
                    self.HorizontalFieldOfView = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadAction.PayloadAction.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "HorizontalFieldOfView":
                self.HorizontalFieldOfView = d[key]

        return

    def get_HorizontalFieldOfView(self):
        return self.HorizontalFieldOfView

    def set_HorizontalFieldOfView(self, value):
        self.HorizontalFieldOfView = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadAction.PayloadAction.toString(self)
        buf += "From CameraAction:\n"
        buf +=    "HorizontalFieldOfView = " + str( self.HorizontalFieldOfView ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if ("CMASI" is None) or ("CMASI" is ""): # this should never happen
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/CameraAction")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("CMASI" + "/CameraAction")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadAction.PayloadAction.toDictMembers(self, d)
        d['HorizontalFieldOfView'] = self.HorizontalFieldOfView

        return

    def getLMCPType(self):
        return self.LMCP_TYPE

    def getSeriesName(self):
        return self.SERIES_NAME

    def getSeriesNameID(self):
        return self.SERIES_NAME_ID

    def getSeriesVersion(self):
        return self.SERIES_VERSION

    def toXMLStr(self, ws):
        str = ws + '<CameraAction Series="CMASI" >\n';
        #str +=PayloadAction.PayloadAction.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</CameraAction>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadAction.PayloadAction.toXMLMembersStr(self, ws)
        buf += ws + "<HorizontalFieldOfView>" + str(self.HorizontalFieldOfView) + "</HorizontalFieldOfView>\n"

        return buf
        
