#!/usr/bin/env python3
#
# Copyright (c) 2016 Intel Corporation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or
# without modification, are permitted provided that the following
# conditions are met:
#
# 1. Redistributions of source code must retain the above
# copyright notice, this list of conditions and the following
# disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of
# its contributors may be used to endorse or promote products
# derived from this software without specific prior written
# permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import xml.etree.ElementTree

NS_GIR = 'http://www.gtk.org/introspection/core/1.0'
NS_C   = 'http://www.gtk.org/introspection/c/1.0'

def exterminate(node, tag):
    nstag = '{' + NS_GIR + '}' + tag
    for p in reversed(node.findall('.//' + nstag + '/..')):
        for i, c in enumerate(p.getchildren()):
            if c.tag != nstag or 'name' in c.attrib:
                continue
            comment1 = xml.etree.ElementTree.Comment(tag)
            comment2 = xml.etree.ElementTree.Comment('/' + tag)
            comment1.tail = c.text
            comment2.tail = c.tail
            p.insert(i, comment2)
            for gc in reversed(c.getchildren()):
                p.insert(i, gc)
            p.insert(i, comment1)
            p.remove(c)

def gir4vapi():
    xml.etree.ElementTree.register_namespace('', NS_GIR)
    xml.etree.ElementTree.register_namespace('c', NS_C)
    gir = xml.etree.ElementTree.parse(sys.stdin)
    exterminate(gir, 'union')
    exterminate(gir, 'record')
    gir.write(sys.stdout.buffer, xml_declaration=True)

if __name__ == '__main__':
    sys.exit(gir4vapi())
