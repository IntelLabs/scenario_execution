# Copyright (C) 2024 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions
# and limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# Generated from OpenSCENARIO2.g4 by ANTLR 4.7.2
from antlr4 import *
from io import StringIO
from typing.io import TextIO
import sys


from antlr4.Token import CommonToken
import re
import importlib
# Allow languages to extend the lexer and parser, by loading the parser dynamically
module_path = __name__[:-5]
language_name = __name__.split('.')[-1]
language_name = language_name[:-5]  # Remove Lexer from name
LanguageParser = getattr(importlib.import_module('{}Parser'.format(module_path)), '{}Parser'.format(language_name))


def serializedATN():
    with StringIO() as buf:
        buf.write("\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\2i")
        buf.write("\u036c\b\1\4\2\t\2\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7")
        buf.write("\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13\t\13\4\f\t\f\4\r\t\r")
        buf.write("\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\4\22\t\22\4\23")
        buf.write("\t\23\4\24\t\24\4\25\t\25\4\26\t\26\4\27\t\27\4\30\t\30")
        buf.write("\4\31\t\31\4\32\t\32\4\33\t\33\4\34\t\34\4\35\t\35\4\36")
        buf.write("\t\36\4\37\t\37\4 \t \4!\t!\4\"\t\"\4#\t#\4$\t$\4%\t%")
        buf.write("\4&\t&\4\'\t\'\4(\t(\4)\t)\4*\t*\4+\t+\4,\t,\4-\t-\4.")
        buf.write("\t.\4/\t/\4\60\t\60\4\61\t\61\4\62\t\62\4\63\t\63\4\64")
        buf.write("\t\64\4\65\t\65\4\66\t\66\4\67\t\67\48\t8\49\t9\4:\t:")
        buf.write("\4;\t;\4<\t<\4=\t=\4>\t>\4?\t?\4@\t@\4A\tA\4B\tB\4C\t")
        buf.write("C\4D\tD\4E\tE\4F\tF\4G\tG\4H\tH\4I\tI\4J\tJ\4K\tK\4L\t")
        buf.write("L\4M\tM\4N\tN\4O\tO\4P\tP\4Q\tQ\4R\tR\4S\tS\4T\tT\4U\t")
        buf.write("U\4V\tV\4W\tW\4X\tX\4Y\tY\4Z\tZ\4[\t[\4\\\t\\\4]\t]\4")
        buf.write("^\t^\4_\t_\4`\t`\4a\ta\4b\tb\4c\tc\4d\td\4e\te\4f\tf\4")
        buf.write("g\tg\4h\th\4i\ti\4j\tj\4k\tk\4l\tl\4m\tm\4n\tn\4o\to\4")
        buf.write("p\tp\4q\tq\4r\tr\4s\ts\4t\tt\4u\tu\3\2\3\2\3\2\3\2\3\2")
        buf.write("\3\2\3\2\3\3\3\3\3\4\3\4\3\4\3\4\3\4\3\5\3\5\3\5\3\6\3")
        buf.write("\6\3\6\3\7\3\7\3\7\3\7\3\7\3\b\3\b\3\b\3\t\3\t\3\n\3\n")
        buf.write("\3\13\3\13\3\13\3\13\3\13\3\13\3\13\3\f\3\f\3\f\3\f\3")
        buf.write("\f\3\f\3\f\3\r\3\r\3\r\3\16\3\16\3\17\3\17\3\20\3\20\3")
        buf.write("\21\3\21\3\22\3\22\3\22\3\22\3\23\3\23\3\23\3\24\3\24")
        buf.write("\3\24\3\24\3\25\3\25\3\25\3\25\3\25\3\26\3\26\3\27\3\27")
        buf.write("\3\30\3\30\3\30\3\31\3\31\3\31\3\31\3\31\3\31\3\31\3\32")
        buf.write("\3\32\3\32\3\32\3\32\3\32\3\32\3\32\3\32\3\33\3\33\3\33")
        buf.write("\3\33\3\33\3\33\3\34\3\34\3\34\3\34\3\34\3\34\3\34\3\34")
        buf.write("\3\34\3\35\3\35\3\35\3\35\3\35\3\35\3\35\3\36\3\36\3\36")
        buf.write("\3\36\3\36\3\36\3\36\3\36\3\36\3\37\3\37\3\37\3\37\3\37")
        buf.write("\3\37\3\37\3 \3 \3 \3 \3 \3 \3 \3!\3!\3!\3!\3!\3\"\3\"")
        buf.write("\3\"\3\"\3#\3#\3#\3#\3#\3$\3$\3$\3$\3$\3$\3%\3%\3%\3%")
        buf.write("\3%\3&\3&\3&\3&\3&\3&\3&\3\'\3\'\3\'\3\'\3\'\3\'\3(\3")
        buf.write("(\3(\3)\3)\3*\3*\3*\3+\3+\3+\3+\3+\3,\3,\3,\3,\3,\3-\3")
        buf.write("-\3-\3-\3-\3-\3-\3-\3.\3.\3.\3.\3.\3.\3/\3/\3/\3/\3\60")
        buf.write("\3\60\3\60\3\60\3\60\3\60\3\60\3\61\3\61\3\61\3\61\3\61")
        buf.write("\3\62\3\62\3\62\3\62\3\62\3\63\3\63\3\63\3\63\3\63\3\63")
        buf.write("\3\63\3\63\3\64\3\64\3\64\3\64\3\64\3\65\3\65\3\65\3\65")
        buf.write("\3\65\3\65\3\65\3\65\3\65\3\65\3\65\3\65\3\65\3\65\3\65")
        buf.write("\3\66\3\66\3\66\3\67\3\67\3\67\38\38\38\38\38\38\38\3")
        buf.write("9\39\39\39\39\39\39\3:\3:\3:\3:\3:\3:\3:\3:\3:\3;\3;\3")
        buf.write(";\3;\3;\3<\3<\3<\3<\3<\3=\3=\3=\3=\3=\3>\3>\3>\3>\3>\3")
        buf.write(">\3?\3?\3?\3?\3@\3@\3@\3A\3A\3A\3A\3A\3A\3A\3A\3A\3A\3")
        buf.write("A\3B\3B\3B\3B\3B\3B\3B\3B\3B\3B\3C\3C\3C\3C\3C\3C\3C\3")
        buf.write("C\3C\3D\3D\3D\3D\3D\3E\3E\3E\3E\3E\3E\3F\3F\3F\3F\3F\3")
        buf.write("F\3F\3G\3G\3G\3G\3G\3G\3H\3H\3I\3I\3I\3J\3J\3J\3K\3K\3")
        buf.write("K\3K\3L\3L\3L\3L\3M\3M\3M\3N\3N\3O\3O\3O\3P\3P\3Q\3Q\3")
        buf.write("Q\3R\3R\3R\3S\3S\3T\3T\3U\3U\3V\3V\3W\3W\3X\3X\3X\3Y\3")
        buf.write("Y\3Y\3Z\3Z\3Z\5Z\u0292\nZ\3Z\3Z\5Z\u0296\nZ\3Z\5Z\u0299")
        buf.write("\nZ\5Z\u029b\nZ\3Z\3Z\3[\3[\3[\3\\\3\\\3\\\3]\3]\3]\3")
        buf.write("^\3^\3^\3_\3_\5_\u02ad\n_\3_\3_\3`\6`\u02b2\n`\r`\16`")
        buf.write("\u02b3\3a\3a\5a\u02b8\na\3a\5a\u02bb\na\3a\3a\3b\5b\u02c0")
        buf.write("\nb\3b\3b\3c\3c\3c\3c\7c\u02c8\nc\fc\16c\u02cb\13c\3c")
        buf.write("\3c\3c\3c\3c\3d\3d\7d\u02d4\nd\fd\16d\u02d7\13d\3d\3d")
        buf.write("\3e\3e\5e\u02dd\ne\3f\3f\7f\u02e1\nf\ff\16f\u02e4\13f")
        buf.write("\3f\3f\3f\7f\u02e9\nf\ff\16f\u02ec\13f\3f\5f\u02ef\nf")
        buf.write("\3g\3g\5g\u02f3\ng\3h\3h\3i\3i\3i\3i\3i\7i\u02fc\ni\f")
        buf.write("i\16i\u02ff\13i\3i\3i\3i\3i\3i\3i\3i\3i\7i\u0309\ni\f")
        buf.write("i\16i\u030c\13i\3i\3i\3i\5i\u0311\ni\3j\3j\5j\u0315\n")
        buf.write("j\3k\3k\3l\3l\3l\3l\5l\u031d\nl\3m\5m\u0320\nm\3m\7m\u0323")
        buf.write("\nm\fm\16m\u0326\13m\3m\3m\6m\u032a\nm\rm\16m\u032b\3")
        buf.write("m\3m\5m\u0330\nm\3m\6m\u0333\nm\rm\16m\u0334\5m\u0337")
        buf.write("\nm\3n\6n\u033a\nn\rn\16n\u033b\3o\3o\3o\3o\6o\u0342\n")
        buf.write("o\ro\16o\u0343\3p\3p\6p\u0348\np\rp\16p\u0349\3q\3q\3")
        buf.write("q\3q\3q\3q\3q\3q\3q\5q\u0355\nq\3r\3r\7r\u0359\nr\fr\16")
        buf.write("r\u035c\13r\3r\3r\6r\u0360\nr\rr\16r\u0361\3r\5r\u0365")
        buf.write("\nr\3s\3s\3t\3t\3u\3u\3\u02c9\2v\3\3\5\4\7\5\t\6\13\7")
        buf.write("\r\b\17\t\21\n\23\13\25\f\27\r\31\16\33\17\35\20\37\21")
        buf.write("!\22#\23%\24\'\25)\26+\27-\30/\31\61\32\63\33\65\34\67")
        buf.write("\359\36;\37= ?!A\"C#E$G%I&K\'M(O)Q*S+U,W-Y.[/]\60_\61")
        buf.write("a\62c\63e\64g\65i\66k\67m8o9q:s;u<w=y>{?}@\177A\u0081")
        buf.write("B\u0083C\u0085D\u0087E\u0089F\u008bG\u008dH\u008fI\u0091")
        buf.write("J\u0093K\u0095L\u0097M\u0099N\u009bO\u009dP\u009fQ\u00a1")
        buf.write("R\u00a3S\u00a5T\u00a7U\u00a9V\u00abW\u00adX\u00afY\u00b1")
        buf.write("Z\u00b3[\u00b5\\\u00b7]\u00b9^\u00bb_\u00bd`\u00bf\2\u00c1")
        buf.write("\2\u00c3\2\u00c5a\u00c7b\u00c9c\u00cb\2\u00cd\2\u00cf")
        buf.write("\2\u00d1\2\u00d3\2\u00d5\2\u00d7\2\u00d9d\u00dbe\u00dd")
        buf.write("f\u00dfg\u00e1h\u00e3i\u00e5\2\u00e7\2\u00e9\2\3\2\r\4")
        buf.write("\2\13\13\"\"\4\2\f\f\16\17\7\2\f\f\17\17$$))^^\3\2^^\4")
        buf.write("\2--//\4\2GGgg\4\2C\\c|\6\2\62;C\\aac|\3\2~~\3\2\62;\5")
        buf.write("\2\62;CHch\2\u0380\2\3\3\2\2\2\2\5\3\2\2\2\2\7\3\2\2\2")
        buf.write("\2\t\3\2\2\2\2\13\3\2\2\2\2\r\3\2\2\2\2\17\3\2\2\2\2\21")
        buf.write("\3\2\2\2\2\23\3\2\2\2\2\25\3\2\2\2\2\27\3\2\2\2\2\31\3")
        buf.write("\2\2\2\2\33\3\2\2\2\2\35\3\2\2\2\2\37\3\2\2\2\2!\3\2\2")
        buf.write("\2\2#\3\2\2\2\2%\3\2\2\2\2\'\3\2\2\2\2)\3\2\2\2\2+\3\2")
        buf.write("\2\2\2-\3\2\2\2\2/\3\2\2\2\2\61\3\2\2\2\2\63\3\2\2\2\2")
        buf.write("\65\3\2\2\2\2\67\3\2\2\2\29\3\2\2\2\2;\3\2\2\2\2=\3\2")
        buf.write("\2\2\2?\3\2\2\2\2A\3\2\2\2\2C\3\2\2\2\2E\3\2\2\2\2G\3")
        buf.write("\2\2\2\2I\3\2\2\2\2K\3\2\2\2\2M\3\2\2\2\2O\3\2\2\2\2Q")
        buf.write("\3\2\2\2\2S\3\2\2\2\2U\3\2\2\2\2W\3\2\2\2\2Y\3\2\2\2\2")
        buf.write("[\3\2\2\2\2]\3\2\2\2\2_\3\2\2\2\2a\3\2\2\2\2c\3\2\2\2")
        buf.write("\2e\3\2\2\2\2g\3\2\2\2\2i\3\2\2\2\2k\3\2\2\2\2m\3\2\2")
        buf.write("\2\2o\3\2\2\2\2q\3\2\2\2\2s\3\2\2\2\2u\3\2\2\2\2w\3\2")
        buf.write("\2\2\2y\3\2\2\2\2{\3\2\2\2\2}\3\2\2\2\2\177\3\2\2\2\2")
        buf.write("\u0081\3\2\2\2\2\u0083\3\2\2\2\2\u0085\3\2\2\2\2\u0087")
        buf.write("\3\2\2\2\2\u0089\3\2\2\2\2\u008b\3\2\2\2\2\u008d\3\2\2")
        buf.write("\2\2\u008f\3\2\2\2\2\u0091\3\2\2\2\2\u0093\3\2\2\2\2\u0095")
        buf.write("\3\2\2\2\2\u0097\3\2\2\2\2\u0099\3\2\2\2\2\u009b\3\2\2")
        buf.write("\2\2\u009d\3\2\2\2\2\u009f\3\2\2\2\2\u00a1\3\2\2\2\2\u00a3")
        buf.write("\3\2\2\2\2\u00a5\3\2\2\2\2\u00a7\3\2\2\2\2\u00a9\3\2\2")
        buf.write("\2\2\u00ab\3\2\2\2\2\u00ad\3\2\2\2\2\u00af\3\2\2\2\2\u00b1")
        buf.write("\3\2\2\2\2\u00b3\3\2\2\2\2\u00b5\3\2\2\2\2\u00b7\3\2\2")
        buf.write("\2\2\u00b9\3\2\2\2\2\u00bb\3\2\2\2\2\u00bd\3\2\2\2\2\u00c5")
        buf.write("\3\2\2\2\2\u00c7\3\2\2\2\2\u00c9\3\2\2\2\2\u00d9\3\2\2")
        buf.write("\2\2\u00db\3\2\2\2\2\u00dd\3\2\2\2\2\u00df\3\2\2\2\2\u00e1")
        buf.write("\3\2\2\2\2\u00e3\3\2\2\2\3\u00eb\3\2\2\2\5\u00f2\3\2\2")
        buf.write("\2\7\u00f4\3\2\2\2\t\u00f9\3\2\2\2\13\u00fc\3\2\2\2\r")
        buf.write("\u00ff\3\2\2\2\17\u0104\3\2\2\2\21\u0107\3\2\2\2\23\u0109")
        buf.write("\3\2\2\2\25\u010b\3\2\2\2\27\u0112\3\2\2\2\31\u0119\3")
        buf.write("\2\2\2\33\u011c\3\2\2\2\35\u011e\3\2\2\2\37\u0120\3\2")
        buf.write("\2\2!\u0122\3\2\2\2#\u0124\3\2\2\2%\u0128\3\2\2\2\'\u012b")
        buf.write("\3\2\2\2)\u012f\3\2\2\2+\u0134\3\2\2\2-\u0136\3\2\2\2")
        buf.write("/\u0138\3\2\2\2\61\u013b\3\2\2\2\63\u0142\3\2\2\2\65\u014b")
        buf.write("\3\2\2\2\67\u0151\3\2\2\29\u015a\3\2\2\2;\u0161\3\2\2")
        buf.write("\2=\u016a\3\2\2\2?\u0171\3\2\2\2A\u0178\3\2\2\2C\u017d")
        buf.write("\3\2\2\2E\u0181\3\2\2\2G\u0186\3\2\2\2I\u018c\3\2\2\2")
        buf.write("K\u0191\3\2\2\2M\u0198\3\2\2\2O\u019e\3\2\2\2Q\u01a1\3")
        buf.write("\2\2\2S\u01a3\3\2\2\2U\u01a6\3\2\2\2W\u01ab\3\2\2\2Y\u01b0")
        buf.write("\3\2\2\2[\u01b8\3\2\2\2]\u01be\3\2\2\2_\u01c2\3\2\2\2")
        buf.write("a\u01c9\3\2\2\2c\u01ce\3\2\2\2e\u01d3\3\2\2\2g\u01db\3")
        buf.write("\2\2\2i\u01e0\3\2\2\2k\u01ef\3\2\2\2m\u01f2\3\2\2\2o\u01f5")
        buf.write("\3\2\2\2q\u01fc\3\2\2\2s\u0203\3\2\2\2u\u020c\3\2\2\2")
        buf.write("w\u0211\3\2\2\2y\u0216\3\2\2\2{\u021b\3\2\2\2}\u0221\3")
        buf.write("\2\2\2\177\u0225\3\2\2\2\u0081\u0228\3\2\2\2\u0083\u0233")
        buf.write("\3\2\2\2\u0085\u023d\3\2\2\2\u0087\u0246\3\2\2\2\u0089")
        buf.write("\u024b\3\2\2\2\u008b\u0251\3\2\2\2\u008d\u0258\3\2\2\2")
        buf.write("\u008f\u025e\3\2\2\2\u0091\u0260\3\2\2\2\u0093\u0263\3")
        buf.write("\2\2\2\u0095\u0266\3\2\2\2\u0097\u026a\3\2\2\2\u0099\u026e")
        buf.write("\3\2\2\2\u009b\u0271\3\2\2\2\u009d\u0273\3\2\2\2\u009f")
        buf.write("\u0276\3\2\2\2\u00a1\u0278\3\2\2\2\u00a3\u027b\3\2\2\2")
        buf.write("\u00a5\u027e\3\2\2\2\u00a7\u0280\3\2\2\2\u00a9\u0282\3")
        buf.write("\2\2\2\u00ab\u0284\3\2\2\2\u00ad\u0286\3\2\2\2\u00af\u0288")
        buf.write("\3\2\2\2\u00b1\u028b\3\2\2\2\u00b3\u029a\3\2\2\2\u00b5")
        buf.write("\u029e\3\2\2\2\u00b7\u02a1\3\2\2\2\u00b9\u02a4\3\2\2\2")
        buf.write("\u00bb\u02a7\3\2\2\2\u00bd\u02ac\3\2\2\2\u00bf\u02b1\3")
        buf.write("\2\2\2\u00c1\u02b5\3\2\2\2\u00c3\u02bf\3\2\2\2\u00c5\u02c3")
        buf.write("\3\2\2\2\u00c7\u02d1\3\2\2\2\u00c9\u02dc\3\2\2\2\u00cb")
        buf.write("\u02ee\3\2\2\2\u00cd\u02f2\3\2\2\2\u00cf\u02f4\3\2\2\2")
        buf.write("\u00d1\u0310\3\2\2\2\u00d3\u0314\3\2\2\2\u00d5\u0316\3")
        buf.write("\2\2\2\u00d7\u031c\3\2\2\2\u00d9\u031f\3\2\2\2\u00db\u0339")
        buf.write("\3\2\2\2\u00dd\u033d\3\2\2\2\u00df\u0345\3\2\2\2\u00e1")
        buf.write("\u0354\3\2\2\2\u00e3\u0364\3\2\2\2\u00e5\u0366\3\2\2\2")
        buf.write("\u00e7\u0368\3\2\2\2\u00e9\u036a\3\2\2\2\u00eb\u00ec\7")
        buf.write("k\2\2\u00ec\u00ed\7o\2\2\u00ed\u00ee\7r\2\2\u00ee\u00ef")
        buf.write("\7q\2\2\u00ef\u00f0\7t\2\2\u00f0\u00f1\7v\2\2\u00f1\4")
        buf.write("\3\2\2\2\u00f2\u00f3\7\60\2\2\u00f3\6\3\2\2\2\u00f4\u00f5")
        buf.write("\7v\2\2\u00f5\u00f6\7{\2\2\u00f6\u00f7\7r\2\2\u00f7\u00f8")
        buf.write("\7g\2\2\u00f8\b\3\2\2\2\u00f9\u00fa\7k\2\2\u00fa\u00fb")
        buf.write("\7u\2\2\u00fb\n\3\2\2\2\u00fc\u00fd\7U\2\2\u00fd\u00fe")
        buf.write("\7K\2\2\u00fe\f\3\2\2\2\u00ff\u0100\7w\2\2\u0100\u0101")
        buf.write("\7p\2\2\u0101\u0102\7k\2\2\u0102\u0103\7v\2\2\u0103\16")
        buf.write("\3\2\2\2\u0104\u0105\7q\2\2\u0105\u0106\7h\2\2\u0106\20")
        buf.write("\3\2\2\2\u0107\u0108\7.\2\2\u0108\22\3\2\2\2\u0109\u010a")
        buf.write("\7<\2\2\u010a\24\3\2\2\2\u010b\u010c\7h\2\2\u010c\u010d")
        buf.write("\7c\2\2\u010d\u010e\7e\2\2\u010e\u010f\7v\2\2\u010f\u0110")
        buf.write("\7q\2\2\u0110\u0111\7t\2\2\u0111\26\3\2\2\2\u0112\u0113")
        buf.write("\7q\2\2\u0113\u0114\7h\2\2\u0114\u0115\7h\2\2\u0115\u0116")
        buf.write("\7u\2\2\u0116\u0117\7g\2\2\u0117\u0118\7v\2\2\u0118\30")
        buf.write("\3\2\2\2\u0119\u011a\7m\2\2\u011a\u011b\7i\2\2\u011b\32")
        buf.write("\3\2\2\2\u011c\u011d\7o\2\2\u011d\34\3\2\2\2\u011e\u011f")
        buf.write("\7u\2\2\u011f\36\3\2\2\2\u0120\u0121\7C\2\2\u0121 \3\2")
        buf.write("\2\2\u0122\u0123\7M\2\2\u0123\"\3\2\2\2\u0124\u0125\7")
        buf.write("o\2\2\u0125\u0126\7q\2\2\u0126\u0127\7n\2\2\u0127$\3\2")
        buf.write("\2\2\u0128\u0129\7e\2\2\u0129\u012a\7f\2\2\u012a&\3\2")
        buf.write("\2\2\u012b\u012c\7t\2\2\u012c\u012d\7c\2\2\u012d\u012e")
        buf.write("\7f\2\2\u012e(\3\2\2\2\u012f\u0130\7g\2\2\u0130\u0131")
        buf.write("\7p\2\2\u0131\u0132\7w\2\2\u0132\u0133\7o\2\2\u0133*\3")
        buf.write("\2\2\2\u0134\u0135\7?\2\2\u0135,\3\2\2\2\u0136\u0137\7")
        buf.write("#\2\2\u0137.\3\2\2\2\u0138\u0139\7?\2\2\u0139\u013a\7")
        buf.write("?\2\2\u013a\60\3\2\2\2\u013b\u013c\7u\2\2\u013c\u013d")
        buf.write("\7v\2\2\u013d\u013e\7t\2\2\u013e\u013f\7w\2\2\u013f\u0140")
        buf.write("\7e\2\2\u0140\u0141\7v\2\2\u0141\62\3\2\2\2\u0142\u0143")
        buf.write("\7k\2\2\u0143\u0144\7p\2\2\u0144\u0145\7j\2\2\u0145\u0146")
        buf.write("\7g\2\2\u0146\u0147\7t\2\2\u0147\u0148\7k\2\2\u0148\u0149")
        buf.write("\7v\2\2\u0149\u014a\7u\2\2\u014a\64\3\2\2\2\u014b\u014c")
        buf.write("\7c\2\2\u014c\u014d\7e\2\2\u014d\u014e\7v\2\2\u014e\u014f")
        buf.write("\7q\2\2\u014f\u0150\7t\2\2\u0150\66\3\2\2\2\u0151\u0152")
        buf.write("\7u\2\2\u0152\u0153\7e\2\2\u0153\u0154\7g\2\2\u0154\u0155")
        buf.write("\7p\2\2\u0155\u0156\7c\2\2\u0156\u0157\7t\2\2\u0157\u0158")
        buf.write("\7k\2\2\u0158\u0159\7q\2\2\u01598\3\2\2\2\u015a\u015b")
        buf.write("\7c\2\2\u015b\u015c\7e\2\2\u015c\u015d\7v\2\2\u015d\u015e")
        buf.write("\7k\2\2\u015e\u015f\7q\2\2\u015f\u0160\7p\2\2\u0160:\3")
        buf.write("\2\2\2\u0161\u0162\7o\2\2\u0162\u0163\7q\2\2\u0163\u0164")
        buf.write("\7f\2\2\u0164\u0165\7k\2\2\u0165\u0166\7h\2\2\u0166\u0167")
        buf.write("\7k\2\2\u0167\u0168\7g\2\2\u0168\u0169\7t\2\2\u0169<\3")
        buf.write("\2\2\2\u016a\u016b\7g\2\2\u016b\u016c\7z\2\2\u016c\u016d")
        buf.write("\7v\2\2\u016d\u016e\7g\2\2\u016e\u016f\7p\2\2\u016f\u0170")
        buf.write("\7f\2\2\u0170>\3\2\2\2\u0171\u0172\7i\2\2\u0172\u0173")
        buf.write("\7n\2\2\u0173\u0174\7q\2\2\u0174\u0175\7d\2\2\u0175\u0176")
        buf.write("\7c\2\2\u0176\u0177\7n\2\2\u0177@\3\2\2\2\u0178\u0179")
        buf.write("\7n\2\2\u0179\u017a\7k\2\2\u017a\u017b\7u\2\2\u017b\u017c")
        buf.write("\7v\2\2\u017cB\3\2\2\2\u017d\u017e\7k\2\2\u017e\u017f")
        buf.write("\7p\2\2\u017f\u0180\7v\2\2\u0180D\3\2\2\2\u0181\u0182")
        buf.write("\7w\2\2\u0182\u0183\7k\2\2\u0183\u0184\7p\2\2\u0184\u0185")
        buf.write("\7v\2\2\u0185F\3\2\2\2\u0186\u0187\7h\2\2\u0187\u0188")
        buf.write("\7n\2\2\u0188\u0189\7q\2\2\u0189\u018a\7c\2\2\u018a\u018b")
        buf.write("\7v\2\2\u018bH\3\2\2\2\u018c\u018d\7d\2\2\u018d\u018e")
        buf.write("\7q\2\2\u018e\u018f\7q\2\2\u018f\u0190\7n\2\2\u0190J\3")
        buf.write("\2\2\2\u0191\u0192\7u\2\2\u0192\u0193\7v\2\2\u0193\u0194")
        buf.write("\7t\2\2\u0194\u0195\7k\2\2\u0195\u0196\7p\2\2\u0196\u0197")
        buf.write("\7i\2\2\u0197L\3\2\2\2\u0198\u0199\7g\2\2\u0199\u019a")
        buf.write("\7x\2\2\u019a\u019b\7g\2\2\u019b\u019c\7p\2\2\u019c\u019d")
        buf.write("\7v\2\2\u019dN\3\2\2\2\u019e\u019f\7k\2\2\u019f\u01a0")
        buf.write("\7h\2\2\u01a0P\3\2\2\2\u01a1\u01a2\7B\2\2\u01a2R\3\2\2")
        buf.write("\2\u01a3\u01a4\7c\2\2\u01a4\u01a5\7u\2\2\u01a5T\3\2\2")
        buf.write("\2\u01a6\u01a7\7t\2\2\u01a7\u01a8\7k\2\2\u01a8\u01a9\7")
        buf.write("u\2\2\u01a9\u01aa\7g\2\2\u01aaV\3\2\2\2\u01ab\u01ac\7")
        buf.write("h\2\2\u01ac\u01ad\7c\2\2\u01ad\u01ae\7n\2\2\u01ae\u01af")
        buf.write("\7n\2\2\u01afX\3\2\2\2\u01b0\u01b1\7g\2\2\u01b1\u01b2")
        buf.write("\7n\2\2\u01b2\u01b3\7c\2\2\u01b3\u01b4\7r\2\2\u01b4\u01b5")
        buf.write("\7u\2\2\u01b5\u01b6\7g\2\2\u01b6\u01b7\7f\2\2\u01b7Z\3")
        buf.write("\2\2\2\u01b8\u01b9\7g\2\2\u01b9\u01ba\7x\2\2\u01ba\u01bb")
        buf.write("\7g\2\2\u01bb\u01bc\7t\2\2\u01bc\u01bd\7{\2\2\u01bd\\")
        buf.write("\3\2\2\2\u01be\u01bf\7x\2\2\u01bf\u01c0\7c\2\2\u01c0\u01c1")
        buf.write("\7t\2\2\u01c1^\3\2\2\2\u01c2\u01c3\7u\2\2\u01c3\u01c4")
        buf.write("\7c\2\2\u01c4\u01c5\7o\2\2\u01c5\u01c6\7r\2\2\u01c6\u01c7")
        buf.write("\7n\2\2\u01c7\u01c8\7g\2\2\u01c8`\3\2\2\2\u01c9\u01ca")
        buf.write("\7y\2\2\u01ca\u01cb\7k\2\2\u01cb\u01cc\7v\2\2\u01cc\u01cd")
        buf.write("\7j\2\2\u01cdb\3\2\2\2\u01ce\u01cf\7m\2\2\u01cf\u01d0")
        buf.write("\7g\2\2\u01d0\u01d1\7g\2\2\u01d1\u01d2\7r\2\2\u01d2d\3")
        buf.write("\2\2\2\u01d3\u01d4\7f\2\2\u01d4\u01d5\7g\2\2\u01d5\u01d6")
        buf.write("\7h\2\2\u01d6\u01d7\7c\2\2\u01d7\u01d8\7w\2\2\u01d8\u01d9")
        buf.write("\7n\2\2\u01d9\u01da\7v\2\2\u01daf\3\2\2\2\u01db\u01dc")
        buf.write("\7j\2\2\u01dc\u01dd\7c\2\2\u01dd\u01de\7t\2\2\u01de\u01df")
        buf.write("\7f\2\2\u01dfh\3\2\2\2\u01e0\u01e1\7t\2\2\u01e1\u01e2")
        buf.write("\7g\2\2\u01e2\u01e3\7o\2\2\u01e3\u01e4\7q\2\2\u01e4\u01e5")
        buf.write("\7x\2\2\u01e5\u01e6\7g\2\2\u01e6\u01e7\7a\2\2\u01e7\u01e8")
        buf.write("\7f\2\2\u01e8\u01e9\7g\2\2\u01e9\u01ea\7h\2\2\u01ea\u01eb")
        buf.write("\7c\2\2\u01eb\u01ec\7w\2\2\u01ec\u01ed\7n\2\2\u01ed\u01ee")
        buf.write("\7v\2\2\u01eej\3\2\2\2\u01ef\u01f0\7q\2\2\u01f0\u01f1")
        buf.write("\7p\2\2\u01f1l\3\2\2\2\u01f2\u01f3\7f\2\2\u01f3\u01f4")
        buf.write("\7q\2\2\u01f4n\3\2\2\2\u01f5\u01f6\7u\2\2\u01f6\u01f7")
        buf.write("\7g\2\2\u01f7\u01f8\7t\2\2\u01f8\u01f9\7k\2\2\u01f9\u01fa")
        buf.write("\7c\2\2\u01fa\u01fb\7n\2\2\u01fbp\3\2\2\2\u01fc\u01fd")
        buf.write("\7q\2\2\u01fd\u01fe\7p\2\2\u01fe\u01ff\7g\2\2\u01ff\u0200")
        buf.write("\7a\2\2\u0200\u0201\7q\2\2\u0201\u0202\7h\2\2\u0202r\3")
        buf.write("\2\2\2\u0203\u0204\7r\2\2\u0204\u0205\7c\2\2\u0205\u0206")
        buf.write("\7t\2\2\u0206\u0207\7c\2\2\u0207\u0208\7n\2\2\u0208\u0209")
        buf.write("\7n\2\2\u0209\u020a\7g\2\2\u020a\u020b\7n\2\2\u020bt\3")
        buf.write("\2\2\2\u020c\u020d\7y\2\2\u020d\u020e\7c\2\2\u020e\u020f")
        buf.write("\7k\2\2\u020f\u0210\7v\2\2\u0210v\3\2\2\2\u0211\u0212")
        buf.write("\7g\2\2\u0212\u0213\7o\2\2\u0213\u0214\7k\2\2\u0214\u0215")
        buf.write("\7v\2\2\u0215x\3\2\2\2\u0216\u0217\7e\2\2\u0217\u0218")
        buf.write("\7c\2\2\u0218\u0219\7n\2\2\u0219\u021a\7n\2\2\u021az\3")
        buf.write("\2\2\2\u021b\u021c\7w\2\2\u021c\u021d\7p\2\2\u021d\u021e")
        buf.write("\7v\2\2\u021e\u021f\7k\2\2\u021f\u0220\7n\2\2\u0220|\3")
        buf.write("\2\2\2\u0221\u0222\7f\2\2\u0222\u0223\7g\2\2\u0223\u0224")
        buf.write("\7h\2\2\u0224~\3\2\2\2\u0225\u0226\7/\2\2\u0226\u0227")
        buf.write("\7@\2\2\u0227\u0080\3\2\2\2\u0228\u0229\7g\2\2\u0229\u022a")
        buf.write("\7z\2\2\u022a\u022b\7r\2\2\u022b\u022c\7t\2\2\u022c\u022d")
        buf.write("\7g\2\2\u022d\u022e\7u\2\2\u022e\u022f\7u\2\2\u022f\u0230")
        buf.write("\7k\2\2\u0230\u0231\7q\2\2\u0231\u0232\7p\2\2\u0232\u0082")
        buf.write("\3\2\2\2\u0233\u0234\7w\2\2\u0234\u0235\7p\2\2\u0235\u0236")
        buf.write("\7f\2\2\u0236\u0237\7g\2\2\u0237\u0238\7h\2\2\u0238\u0239")
        buf.write("\7k\2\2\u0239\u023a\7p\2\2\u023a\u023b\7g\2\2\u023b\u023c")
        buf.write("\7f\2\2\u023c\u0084\3\2\2\2\u023d\u023e\7g\2\2\u023e\u023f")
        buf.write("\7z\2\2\u023f\u0240\7v\2\2\u0240\u0241\7g\2\2\u0241\u0242")
        buf.write("\7t\2\2\u0242\u0243\7p\2\2\u0243\u0244\7c\2\2\u0244\u0245")
        buf.write("\7n\2\2\u0245\u0086\3\2\2\2\u0246\u0247\7q\2\2\u0247\u0248")
        buf.write("\7p\2\2\u0248\u0249\7n\2\2\u0249\u024a\7{\2\2\u024a\u0088")
        buf.write("\3\2\2\2\u024b\u024c\7e\2\2\u024c\u024d\7q\2\2\u024d\u024e")
        buf.write("\7x\2\2\u024e\u024f\7g\2\2\u024f\u0250\7t\2\2\u0250\u008a")
        buf.write("\3\2\2\2\u0251\u0252\7t\2\2\u0252\u0253\7g\2\2\u0253\u0254")
        buf.write("\7e\2\2\u0254\u0255\7q\2\2\u0255\u0256\7t\2\2\u0256\u0257")
        buf.write("\7f\2\2\u0257\u008c\3\2\2\2\u0258\u0259\7t\2\2\u0259\u025a")
        buf.write("\7c\2\2\u025a\u025b\7p\2\2\u025b\u025c\7i\2\2\u025c\u025d")
        buf.write("\7g\2\2\u025d\u008e\3\2\2\2\u025e\u025f\7A\2\2\u025f\u0090")
        buf.write("\3\2\2\2\u0260\u0261\7?\2\2\u0261\u0262\7@\2\2\u0262\u0092")
        buf.write("\3\2\2\2\u0263\u0264\7q\2\2\u0264\u0265\7t\2\2\u0265\u0094")
        buf.write("\3\2\2\2\u0266\u0267\7c\2\2\u0267\u0268\7p\2\2\u0268\u0269")
        buf.write("\7f\2\2\u0269\u0096\3\2\2\2\u026a\u026b\7p\2\2\u026b\u026c")
        buf.write("\7q\2\2\u026c\u026d\7v\2\2\u026d\u0098\3\2\2\2\u026e\u026f")
        buf.write("\7#\2\2\u026f\u0270\7?\2\2\u0270\u009a\3\2\2\2\u0271\u0272")
        buf.write("\7>\2\2\u0272\u009c\3\2\2\2\u0273\u0274\7>\2\2\u0274\u0275")
        buf.write("\7?\2\2\u0275\u009e\3\2\2\2\u0276\u0277\7@\2\2\u0277\u00a0")
        buf.write("\3\2\2\2\u0278\u0279\7@\2\2\u0279\u027a\7?\2\2\u027a\u00a2")
        buf.write("\3\2\2\2\u027b\u027c\7k\2\2\u027c\u027d\7p\2\2\u027d\u00a4")
        buf.write("\3\2\2\2\u027e\u027f\7-\2\2\u027f\u00a6\3\2\2\2\u0280")
        buf.write("\u0281\7/\2\2\u0281\u00a8\3\2\2\2\u0282\u0283\7,\2\2\u0283")
        buf.write("\u00aa\3\2\2\2\u0284\u0285\7\61\2\2\u0285\u00ac\3\2\2")
        buf.write("\2\u0286\u0287\7\'\2\2\u0287\u00ae\3\2\2\2\u0288\u0289")
        buf.write("\7k\2\2\u0289\u028a\7v\2\2\u028a\u00b0\3\2\2\2\u028b\u028c")
        buf.write("\7\60\2\2\u028c\u028d\7\60\2\2\u028d\u00b2\3\2\2\2\u028e")
        buf.write("\u028f\6Z\2\2\u028f\u029b\5\u00bf`\2\u0290\u0292\7\17")
        buf.write("\2\2\u0291\u0290\3\2\2\2\u0291\u0292\3\2\2\2\u0292\u0293")
        buf.write("\3\2\2\2\u0293\u0296\7\f\2\2\u0294\u0296\4\16\17\2\u0295")
        buf.write("\u0291\3\2\2\2\u0295\u0294\3\2\2\2\u0296\u0298\3\2\2\2")
        buf.write("\u0297\u0299\5\u00bf`\2\u0298\u0297\3\2\2\2\u0298\u0299")
        buf.write("\3\2\2\2\u0299\u029b\3\2\2\2\u029a\u028e\3\2\2\2\u029a")
        buf.write("\u0295\3\2\2\2\u029b\u029c\3\2\2\2\u029c\u029d\bZ\2\2")
        buf.write("\u029d\u00b4\3\2\2\2\u029e\u029f\7]\2\2\u029f\u02a0\b")
        buf.write("[\3\2\u02a0\u00b6\3\2\2\2\u02a1\u02a2\7_\2\2\u02a2\u02a3")
        buf.write("\b\\\4\2\u02a3\u00b8\3\2\2\2\u02a4\u02a5\7*\2\2\u02a5")
        buf.write("\u02a6\b]\5\2\u02a6\u00ba\3\2\2\2\u02a7\u02a8\7+\2\2\u02a8")
        buf.write("\u02a9\b^\6\2\u02a9\u00bc\3\2\2\2\u02aa\u02ad\5\u00bf")
        buf.write("`\2\u02ab\u02ad\5\u00c1a\2\u02ac\u02aa\3\2\2\2\u02ac\u02ab")
        buf.write("\3\2\2\2\u02ad\u02ae\3\2\2\2\u02ae\u02af\b_\7\2\u02af")
        buf.write("\u00be\3\2\2\2\u02b0\u02b2\t\2\2\2\u02b1\u02b0\3\2\2\2")
        buf.write("\u02b2\u02b3\3\2\2\2\u02b3\u02b1\3\2\2\2\u02b3\u02b4\3")
        buf.write("\2\2\2\u02b4\u00c0\3\2\2\2\u02b5\u02b7\7^\2\2\u02b6\u02b8")
        buf.write("\5\u00bf`\2\u02b7\u02b6\3\2\2\2\u02b7\u02b8\3\2\2\2\u02b8")
        buf.write("\u02ba\3\2\2\2\u02b9\u02bb\7\17\2\2\u02ba\u02b9\3\2\2")
        buf.write("\2\u02ba\u02bb\3\2\2\2\u02bb\u02bc\3\2\2\2\u02bc\u02bd")
        buf.write("\7\f\2\2\u02bd\u00c2\3\2\2\2\u02be\u02c0\7\17\2\2\u02bf")
        buf.write("\u02be\3\2\2\2\u02bf\u02c0\3\2\2\2\u02c0\u02c1\3\2\2\2")
        buf.write("\u02c1\u02c2\7\f\2\2\u02c2\u00c4\3\2\2\2\u02c3\u02c4\7")
        buf.write("\61\2\2\u02c4\u02c5\7,\2\2\u02c5\u02c9\3\2\2\2\u02c6\u02c8")
        buf.write("\13\2\2\2\u02c7\u02c6\3\2\2\2\u02c8\u02cb\3\2\2\2\u02c9")
        buf.write("\u02ca\3\2\2\2\u02c9\u02c7\3\2\2\2\u02ca\u02cc\3\2\2\2")
        buf.write("\u02cb\u02c9\3\2\2\2\u02cc\u02cd\7,\2\2\u02cd\u02ce\7")
        buf.write("\61\2\2\u02ce\u02cf\3\2\2\2\u02cf\u02d0\bc\7\2\u02d0\u00c6")
        buf.write("\3\2\2\2\u02d1\u02d5\7%\2\2\u02d2\u02d4\n\3\2\2\u02d3")
        buf.write("\u02d2\3\2\2\2\u02d4\u02d7\3\2\2\2\u02d5\u02d3\3\2\2\2")
        buf.write("\u02d5\u02d6\3\2\2\2\u02d6\u02d8\3\2\2\2\u02d7\u02d5\3")
        buf.write("\2\2\2\u02d8\u02d9\bd\7\2\u02d9\u00c8\3\2\2\2\u02da\u02dd")
        buf.write("\5\u00cbf\2\u02db\u02dd\5\u00d1i\2\u02dc\u02da\3\2\2\2")
        buf.write("\u02dc\u02db\3\2\2\2\u02dd\u00ca\3\2\2\2\u02de\u02e2\7")
        buf.write("$\2\2\u02df\u02e1\5\u00cdg\2\u02e0\u02df\3\2\2\2\u02e1")
        buf.write("\u02e4\3\2\2\2\u02e2\u02e0\3\2\2\2\u02e2\u02e3\3\2\2\2")
        buf.write("\u02e3\u02e5\3\2\2\2\u02e4\u02e2\3\2\2\2\u02e5\u02ef\7")
        buf.write("$\2\2\u02e6\u02ea\7)\2\2\u02e7\u02e9\5\u00cdg\2\u02e8")
        buf.write("\u02e7\3\2\2\2\u02e9\u02ec\3\2\2\2\u02ea\u02e8\3\2\2\2")
        buf.write("\u02ea\u02eb\3\2\2\2\u02eb\u02ed\3\2\2\2\u02ec\u02ea\3")
        buf.write("\2\2\2\u02ed\u02ef\7)\2\2\u02ee\u02de\3\2\2\2\u02ee\u02e6")
        buf.write("\3\2\2\2\u02ef\u00cc\3\2\2\2\u02f0\u02f3\5\u00cfh\2\u02f1")
        buf.write("\u02f3\5\u00d7l\2\u02f2\u02f0\3\2\2\2\u02f2\u02f1\3\2")
        buf.write("\2\2\u02f3\u00ce\3\2\2\2\u02f4\u02f5\n\4\2\2\u02f5\u00d0")
        buf.write("\3\2\2\2\u02f6\u02f7\7$\2\2\u02f7\u02f8\7$\2\2\u02f8\u02f9")
        buf.write("\7$\2\2\u02f9\u02fd\3\2\2\2\u02fa\u02fc\5\u00d3j\2\u02fb")
        buf.write("\u02fa\3\2\2\2\u02fc\u02ff\3\2\2\2\u02fd\u02fb\3\2\2\2")
        buf.write("\u02fd\u02fe\3\2\2\2\u02fe\u0300\3\2\2\2\u02ff\u02fd\3")
        buf.write("\2\2\2\u0300\u0301\7$\2\2\u0301\u0302\7$\2\2\u0302\u0311")
        buf.write("\7$\2\2\u0303\u0304\7)\2\2\u0304\u0305\7)\2\2\u0305\u0306")
        buf.write("\7)\2\2\u0306\u030a\3\2\2\2\u0307\u0309\5\u00d3j\2\u0308")
        buf.write("\u0307\3\2\2\2\u0309\u030c\3\2\2\2\u030a\u0308\3\2\2\2")
        buf.write("\u030a\u030b\3\2\2\2\u030b\u030d\3\2\2\2\u030c\u030a\3")
        buf.write("\2\2\2\u030d\u030e\7)\2\2\u030e\u030f\7)\2\2\u030f\u0311")
        buf.write("\7)\2\2\u0310\u02f6\3\2\2\2\u0310\u0303\3\2\2\2\u0311")
        buf.write("\u00d2\3\2\2\2\u0312\u0315\5\u00d5k\2\u0313\u0315\5\u00d7")
        buf.write("l\2\u0314\u0312\3\2\2\2\u0314\u0313\3\2\2\2\u0315\u00d4")
        buf.write("\3\2\2\2\u0316\u0317\n\5\2\2\u0317\u00d6\3\2\2\2\u0318")
        buf.write("\u0319\7^\2\2\u0319\u031d\13\2\2\2\u031a\u031b\7^\2\2")
        buf.write("\u031b\u031d\5\u00c3b\2\u031c\u0318\3\2\2\2\u031c\u031a")
        buf.write("\3\2\2\2\u031d\u00d8\3\2\2\2\u031e\u0320\t\6\2\2\u031f")
        buf.write("\u031e\3\2\2\2\u031f\u0320\3\2\2\2\u0320\u0324\3\2\2\2")
        buf.write("\u0321\u0323\5\u00e7t\2\u0322\u0321\3\2\2\2\u0323\u0326")
        buf.write("\3\2\2\2\u0324\u0322\3\2\2\2\u0324\u0325\3\2\2\2\u0325")
        buf.write("\u0327\3\2\2\2\u0326\u0324\3\2\2\2\u0327\u0329\7\60\2")
        buf.write("\2\u0328\u032a\5\u00e7t\2\u0329\u0328\3\2\2\2\u032a\u032b")
        buf.write("\3\2\2\2\u032b\u0329\3\2\2\2\u032b\u032c\3\2\2\2\u032c")
        buf.write("\u0336\3\2\2\2\u032d\u032f\t\7\2\2\u032e\u0330\t\6\2\2")
        buf.write("\u032f\u032e\3\2\2\2\u032f\u0330\3\2\2\2\u0330\u0332\3")
        buf.write("\2\2\2\u0331\u0333\5\u00e7t\2\u0332\u0331\3\2\2\2\u0333")
        buf.write("\u0334\3\2\2\2\u0334\u0332\3\2\2\2\u0334\u0335\3\2\2\2")
        buf.write("\u0335\u0337\3\2\2\2\u0336\u032d\3\2\2\2\u0336\u0337\3")
        buf.write("\2\2\2\u0337\u00da\3\2\2\2\u0338\u033a\5\u00e7t\2\u0339")
        buf.write("\u0338\3\2\2\2\u033a\u033b\3\2\2\2\u033b\u0339\3\2\2\2")
        buf.write("\u033b\u033c\3\2\2\2\u033c\u00dc\3\2\2\2\u033d\u033e\7")
        buf.write("\62\2\2\u033e\u033f\7z\2\2\u033f\u0341\3\2\2\2\u0340\u0342")
        buf.write("\5\u00e9u\2\u0341\u0340\3\2\2\2\u0342\u0343\3\2\2\2\u0343")
        buf.write("\u0341\3\2\2\2\u0343\u0344\3\2\2\2\u0344\u00de\3\2\2\2")
        buf.write("\u0345\u0347\7/\2\2\u0346\u0348\5\u00e7t\2\u0347\u0346")
        buf.write("\3\2\2\2\u0348\u0349\3\2\2\2\u0349\u0347\3\2\2\2\u0349")
        buf.write("\u034a\3\2\2\2\u034a\u00e0\3\2\2\2\u034b\u034c\7v\2\2")
        buf.write("\u034c\u034d\7t\2\2\u034d\u034e\7w\2\2\u034e\u0355\7g")
        buf.write("\2\2\u034f\u0350\7h\2\2\u0350\u0351\7c\2\2\u0351\u0352")
        buf.write("\7n\2\2\u0352\u0353\7u\2\2\u0353\u0355\7g\2\2\u0354\u034b")
        buf.write("\3\2\2\2\u0354\u034f\3\2\2\2\u0355\u00e2\3\2\2\2\u0356")
        buf.write("\u035a\t\b\2\2\u0357\u0359\t\t\2\2\u0358\u0357\3\2\2\2")
        buf.write("\u0359\u035c\3\2\2\2\u035a\u0358\3\2\2\2\u035a\u035b\3")
        buf.write("\2\2\2\u035b\u0365\3\2\2\2\u035c\u035a\3\2\2\2\u035d\u035f")
        buf.write("\7~\2\2\u035e\u0360\n\n\2\2\u035f\u035e\3\2\2\2\u0360")
        buf.write("\u0361\3\2\2\2\u0361\u035f\3\2\2\2\u0361\u0362\3\2\2\2")
        buf.write("\u0362\u0363\3\2\2\2\u0363\u0365\7~\2\2\u0364\u0356\3")
        buf.write("\2\2\2\u0364\u035d\3\2\2\2\u0365\u00e4\3\2\2\2\u0366\u0367")
        buf.write("\n\n\2\2\u0367\u00e6\3\2\2\2\u0368\u0369\t\13\2\2\u0369")
        buf.write("\u00e8\3\2\2\2\u036a\u036b\t\f\2\2\u036b\u00ea\3\2\2\2")
        buf.write("%\2\u0291\u0295\u0298\u029a\u02ac\u02b3\u02b7\u02ba\u02bf")
        buf.write("\u02c9\u02d5\u02dc\u02e2\u02ea\u02ee\u02f2\u02fd\u030a")
        buf.write("\u0310\u0314\u031c\u031f\u0324\u032b\u032f\u0334\u0336")
        buf.write("\u033b\u0343\u0349\u0354\u035a\u0361\u0364\b\3Z\2\3[\3")
        buf.write("\3\\\4\3]\5\3^\6\b\2\2")
        return buf.getvalue()


class OpenSCENARIO2Lexer(Lexer):

    atn = ATNDeserializer().deserialize(serializedATN())

    decisionsToDFA = [DFA(ds, i) for i, ds in enumerate(atn.decisionToState)]

    T__0 = 1
    T__1 = 2
    T__2 = 3
    T__3 = 4
    T__4 = 5
    T__5 = 6
    T__6 = 7
    T__7 = 8
    T__8 = 9
    T__9 = 10
    T__10 = 11
    T__11 = 12
    T__12 = 13
    T__13 = 14
    T__14 = 15
    T__15 = 16
    T__16 = 17
    T__17 = 18
    T__18 = 19
    T__19 = 20
    T__20 = 21
    T__21 = 22
    T__22 = 23
    T__23 = 24
    T__24 = 25
    T__25 = 26
    T__26 = 27
    T__27 = 28
    T__28 = 29
    T__29 = 30
    T__30 = 31
    T__31 = 32
    T__32 = 33
    T__33 = 34
    T__34 = 35
    T__35 = 36
    T__36 = 37
    T__37 = 38
    T__38 = 39
    T__39 = 40
    T__40 = 41
    T__41 = 42
    T__42 = 43
    T__43 = 44
    T__44 = 45
    T__45 = 46
    T__46 = 47
    T__47 = 48
    T__48 = 49
    T__49 = 50
    T__50 = 51
    T__51 = 52
    T__52 = 53
    T__53 = 54
    T__54 = 55
    T__55 = 56
    T__56 = 57
    T__57 = 58
    T__58 = 59
    T__59 = 60
    T__60 = 61
    T__61 = 62
    T__62 = 63
    T__63 = 64
    T__64 = 65
    T__65 = 66
    T__66 = 67
    T__67 = 68
    T__68 = 69
    T__69 = 70
    T__70 = 71
    T__71 = 72
    T__72 = 73
    T__73 = 74
    T__74 = 75
    T__75 = 76
    T__76 = 77
    T__77 = 78
    T__78 = 79
    T__79 = 80
    T__80 = 81
    T__81 = 82
    T__82 = 83
    T__83 = 84
    T__84 = 85
    T__85 = 86
    T__86 = 87
    T__87 = 88
    NEWLINE = 89
    OPEN_BRACK = 90
    CLOSE_BRACK = 91
    OPEN_PAREN = 92
    CLOSE_PAREN = 93
    SKIP_ = 94
    BLOCK_COMMENT = 95
    LINE_COMMENT = 96
    StringLiteral = 97
    FloatLiteral = 98
    UintLiteral = 99
    HexUintLiteral = 100
    IntLiteral = 101
    BoolLiteral = 102
    Identifier = 103

    channelNames = [u"DEFAULT_TOKEN_CHANNEL", u"HIDDEN"]

    modeNames = ["DEFAULT_MODE"]

    literalNames = ["<INVALID>",
                    "'import'", "'.'", "'type'", "'is'", "'SI'", "'unit'", "'of'",
                    "','", "':'", "'factor'", "'offset'", "'kg'", "'m'", "'s'",
                    "'A'", "'K'", "'mol'", "'cd'", "'rad'", "'enum'", "'='", "'!'",
                    "'=='", "'struct'", "'inherits'", "'actor'", "'scenario'", "'action'",
                    "'modifier'", "'extend'", "'global'", "'list'", "'int'", "'uint'",
                    "'float'", "'bool'", "'string'", "'event'", "'if'", "'@'", "'as'",
                    "'rise'", "'fall'", "'elapsed'", "'every'", "'var'", "'sample'",
                    "'with'", "'keep'", "'default'", "'hard'", "'remove_default'",
                    "'on'", "'do'", "'serial'", "'one_of'", "'parallel'", "'wait'",
                    "'emit'", "'call'", "'until'", "'def'", "'->'", "'expression'",
                    "'undefined'", "'external'", "'only'", "'cover'", "'record'",
                    "'range'", "'?'", "'=>'", "'or'", "'and'", "'not'", "'!='",
                    "'<'", "'<='", "'>'", "'>='", "'in'", "'+'", "'-'", "'*'", "'/'",
                    "'%'", "'it'", "'..'", "'['", "']'", "'('", "')'"]

    symbolicNames = ["<INVALID>",
                     "NEWLINE", "OPEN_BRACK", "CLOSE_BRACK", "OPEN_PAREN", "CLOSE_PAREN",
                     "SKIP_", "BLOCK_COMMENT", "LINE_COMMENT", "StringLiteral", "FloatLiteral",
                     "UintLiteral", "HexUintLiteral", "IntLiteral", "BoolLiteral",
                     "Identifier"]

    ruleNames = ["T__0", "T__1", "T__2", "T__3", "T__4", "T__5", "T__6",
                 "T__7", "T__8", "T__9", "T__10", "T__11", "T__12", "T__13",
                 "T__14", "T__15", "T__16", "T__17", "T__18", "T__19",
                 "T__20", "T__21", "T__22", "T__23", "T__24", "T__25",
                 "T__26", "T__27", "T__28", "T__29", "T__30", "T__31",
                 "T__32", "T__33", "T__34", "T__35", "T__36", "T__37",
                 "T__38", "T__39", "T__40", "T__41", "T__42", "T__43",
                 "T__44", "T__45", "T__46", "T__47", "T__48", "T__49",
                 "T__50", "T__51", "T__52", "T__53", "T__54", "T__55",
                 "T__56", "T__57", "T__58", "T__59", "T__60", "T__61",
                 "T__62", "T__63", "T__64", "T__65", "T__66", "T__67",
                 "T__68", "T__69", "T__70", "T__71", "T__72", "T__73",
                 "T__74", "T__75", "T__76", "T__77", "T__78", "T__79",
                 "T__80", "T__81", "T__82", "T__83", "T__84", "T__85",
                 "T__86", "T__87", "NEWLINE", "OPEN_BRACK", "CLOSE_BRACK",
                 "OPEN_PAREN", "CLOSE_PAREN", "SKIP_", "SPACES", "LINE_JOINING",
                 "RN", "BLOCK_COMMENT", "LINE_COMMENT", "StringLiteral",
                 "Shortstring", "ShortstringElem", "ShortstringChar", "Longstring",
                 "LongstringElem", "LongstringChar", "StringEscapeSeq",
                 "FloatLiteral", "UintLiteral", "HexUintLiteral", "IntLiteral",
                 "BoolLiteral", "Identifier", "NonVerticalLineChar", "Digit",
                 "HexDigit"]

    grammarFileName = "OpenSCENARIO2.g4"

    def __init__(self, input=None, output: TextIO = sys.stdout):
        super().__init__(input, output)
        self.checkVersion("4.7.2")
        self._interp = LexerATNSimulator(self, self.atn, self.decisionsToDFA, PredictionContextCache())
        self._actions = None
        self._predicates = None

    @property
    def tokens(self):
        try:
            return self._tokens
        except AttributeError:
            self._tokens = []
            return self._tokens

    @property
    def indents(self):
        try:
            return self._indents
        except AttributeError:
            self._indents = []
            return self._indents

    @property
    def opened(self):
        try:
            return self._opened
        except AttributeError:
            self._opened = 0
            return self._opened

    @opened.setter
    def opened(self, value):
        self._opened = value

    @property
    def lastToken(self):
        try:
            return self._lastToken
        except AttributeError:
            self._lastToken = None
            return self._lastToken

    @lastToken.setter
    def lastToken(self, value):
        self._lastToken = value

    def reset(self):
        super().reset()
        self.tokens = []
        self.indents = []
        self.opened = 0
        self.lastToken = None

    def emitToken(self, t):
        super().emitToken(t)
        self.tokens.append(t)

    def nextToken(self):
        if self._input.LA(1) == Token.EOF and self.indents:
            for i in range(len(self.tokens)-1, -1, -1):
                if self.tokens[i].type == Token.EOF:
                    self.tokens.pop(i)
            self.emitToken(self.commonToken(LanguageParser.NEWLINE, '\n'))
            while self.indents:
                self.emitToken(self.createDedent())
                self.indents.pop()
            self.emitToken(self.commonToken(LanguageParser.EOF, "<EOF>"))
        next = super().nextToken()
        if next.channel == Token.DEFAULT_CHANNEL:
            self.lastToken = next
        return next if not self.tokens else self.tokens.pop(0)

    def createDedent(self):
        dedent = self.commonToken(LanguageParser.DEDENT, "")
        dedent.line = self.lastToken.line
        return dedent

    def commonToken(self, type, text, indent=0):
        stop = self.getCharIndex()-1-indent
        start = (stop - len(text) + 1) if text else stop
        return CommonToken(self._tokenFactorySourcePair, type, super().DEFAULT_TOKEN_CHANNEL, start, stop)

    @staticmethod
    def getIndentationCount(spaces):
        count = 0
        for ch in spaces:
            if ch == '\t':
                count += 8 - (count % 8)
            else:
                count += 1
        return count

    def atStartOfInput(self):
        return Lexer.column.fget(self) == 0 and Lexer.line.fget(self) == 1

    def action(self, localctx: RuleContext, ruleIndex: int, actionIndex: int):
        if self._actions is None:
            actions = dict()
            actions[88] = self.NEWLINE_action
            actions[89] = self.OPEN_BRACK_action
            actions[90] = self.CLOSE_BRACK_action
            actions[91] = self.OPEN_PAREN_action
            actions[92] = self.CLOSE_PAREN_action
            self._actions = actions
        action = self._actions.get(ruleIndex, None)
        if action is not None:
            action(localctx, actionIndex)
        else:
            raise Exception("No registered action for:" + str(ruleIndex))

    def NEWLINE_action(self, localctx: RuleContext, actionIndex: int):
        if actionIndex == 0:

            tempt = Lexer.text.fget(self)
            newLine = re.sub("[^\r\n\f]+", "", tempt)
            spaces = re.sub("[\r\n\f]+", "", tempt)
            la_char = ""
            try:
                la = self._input.LA(1)
                la_char = chr(la)       # Python does not compare char to ints directly
            except ValueError:          # End of file
                pass
            # Strip newlines inside open clauses except if we are near EOF. We keep NEWLINEs near EOF to
            # satisfy the final newline needed by the single_put rule used by the REPL.
            try:
                nextnext_la = self._input.LA(2)
                nextnext_la_char = chr(nextnext_la)
            except ValueError:
                nextnext_eof = True
            else:
                nextnext_eof = False
            if self.opened > 0 or nextnext_eof is False and (la_char == '\r' or la_char == '\n' or la_char == '\f' or la_char == '#'):
                self.skip()
            else:
                indent = self.getIndentationCount(spaces)
                previous = self.indents[-1] if self.indents else 0
                self.emitToken(self.commonToken(self.NEWLINE, newLine, indent=indent))      # NEWLINE is actually the '\n' char
                if indent == previous:
                    self.skip()
                elif indent > previous:
                    self.indents.append(indent)
                    self.emitToken(self.commonToken(LanguageParser.INDENT, spaces))
                else:
                    while self.indents and self.indents[-1] > indent:
                        self.emitToken(self.createDedent())
                        self.indents.pop()

    def OPEN_BRACK_action(self, localctx: RuleContext, actionIndex: int):
        if actionIndex == 1:
            self.opened += 1

    def CLOSE_BRACK_action(self, localctx: RuleContext, actionIndex: int):
        if actionIndex == 2:
            self.opened -= 1

    def OPEN_PAREN_action(self, localctx: RuleContext, actionIndex: int):
        if actionIndex == 3:
            self.opened += 1

    def CLOSE_PAREN_action(self, localctx: RuleContext, actionIndex: int):
        if actionIndex == 4:
            self.opened -= 1

    def sempred(self, localctx: RuleContext, ruleIndex: int, predIndex: int):
        if self._predicates is None:
            preds = dict()
            preds[88] = self.NEWLINE_sempred
            self._predicates = preds
        pred = self._predicates.get(ruleIndex, None)
        if pred is not None:
            return pred(localctx, predIndex)
        else:
            raise Exception("No registered predicate for:" + str(ruleIndex))

    def NEWLINE_sempred(self, localctx: RuleContext, predIndex: int):
        if predIndex == 0:
            return self.atStartOfInput()
