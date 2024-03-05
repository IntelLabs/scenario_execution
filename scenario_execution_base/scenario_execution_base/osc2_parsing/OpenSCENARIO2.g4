// MIT License

// Copyright (c) 2018 CARLA

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//  Copyright (C) 2024 Intel Corporation

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing,
//  software distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions
//  and limitations under the License.

//  SPDX-License-Identifier: Apache-2.0

grammar OpenSCENARIO2;

tokens { INDENT, DEDENT}


@lexer::header{
from antlr4.Token import CommonToken
import re
import importlib
# Allow languages to extend the lexer and parser, by loading the parser dynamically
module_path = __name__[:-5]
language_name = __name__.split('.')[-1]
language_name = language_name[:-5]  # Remove Lexer from name
LanguageParser = getattr(importlib.import_module('{}Parser'.format(module_path)), '{}Parser'.format(language_name))
}

@lexer::members {

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
        for i in range(len(self.tokens)-1,-1,-1):
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

}


//----------------------------------------
// Parser rules
/*Top-Level structure*/
osc_file : preludeStatement* oscDeclaration* EOF; 

preludeStatement : importStatement  ;

importStatement
	: 'import' importReference NEWLINE
	|  NEWLINE;

importReference
	:   StringLiteral
	|   structuredIdentifier
	;

structuredIdentifier
	:   Identifier
	|   structuredIdentifier'.'Identifier
	;

oscDeclaration
	:   physicalTypeDeclaration 
	|   unitDeclaration
	|   enumDeclaration
	|   structDeclaration   
	|   actorDeclaration
	|   actionDeclaration
	|   scenarioDeclaration
	|   modifierDeclaration
	|   typeExtension
	|   globalParameterDeclaration
	|   NEWLINE
	;

//----------------------------------------
// physicalTypeDeclaration
physicalTypeDeclaration : 'type' physicalTypeName 'is' baseUnitSpecifier NEWLINE;

physicalTypeName : Identifier;

baseUnitSpecifier : sIBaseUnitSpecifier;

sIBaseUnitSpecifier : 'SI' OPEN_PAREN siBaseExponentList CLOSE_PAREN;

//----------------------------------------
// unitDeclaration
unitDeclaration : 'unit' unitName 'of' physicalTypeName 'is' unitSpecifier NEWLINE;

unitSpecifier : siUnitSpecifier  ;
unitName : Identifier | siBaseUnitName ;

siBaseExponentList : siBaseExponent (',' siBaseExponent)*  ;
siBaseExponent : siBaseUnitName ':' integerLiteral  ;

siUnitSpecifier : 'SI' '(' siBaseExponentList (',' siFactor)? (',' siOffset)? ')'  ;
siFactor : 'factor' ':' ( FloatLiteral | integerLiteral ) ;
siOffset : 'offset' ':' ( FloatLiteral | integerLiteral ) ;
siBaseUnitName : 'kg' | 'm' | 's' | 'A' | 'K' | 'mol' | 'cd' | 'rad'  ;

//----------------------------------------
// enumDeclaration
enumDeclaration : 'enum' enumName ':' OPEN_BRACK  enumMemberDecl (','  enumMemberDecl)* CLOSE_BRACK NEWLINE;

enumMemberDecl : enumMemberName ('=' enumMemberValue )?;

enumMemberValue: UintLiteral | HexUintLiteral;

enumName : Identifier;

enumMemberName : Identifier;

enumValueReference : enumName '!' enumMemberName;

//----------------------------------------
inheritsCondition : OPEN_PAREN fieldName '==' (enumValueReference | BoolLiteral) CLOSE_PAREN ;

//----------------------------------------
// structDeclaration
structDeclaration : 
	'struct' structName (structInherits)? 
	((':' NEWLINE INDENT structMemberDecl+ DEDENT) | NEWLINE);

structInherits : 'inherits' structName (inheritsCondition)? ;

structMemberDecl 
	:  eventDeclaration 
	|  fieldDeclaration 
	|  constraintDeclaration 
	|  methodDeclaration
	|  coverageDeclaration;

fieldName : Identifier;

structName : Identifier;

//----------------------------------------
// actorDeclaration
actorDeclaration :
	'actor' actorName (actorInherits)?
	((':' NEWLINE INDENT actorMemberDecl+ DEDENT) | NEWLINE);

actorInherits : 'inherits' actorName (inheritsCondition)? ;

actorMemberDecl
	:  eventDeclaration 
	|  fieldDeclaration 
	|  constraintDeclaration 
	|  methodDeclaration
	|  coverageDeclaration;

actorName : Identifier;

//----------------------------------------
// scenarioDeclaration
scenarioDeclaration 
	: 'scenario' qualifiedBehaviorName (scenarioInherits)?
	((':' NEWLINE INDENT
		(scenarioMemberDecl | behaviorSpecification )+
		DEDENT) | NEWLINE);

scenarioInherits : 'inherits' qualifiedBehaviorName (inheritsCondition)? ;

scenarioMemberDecl 
	: eventDeclaration 
	| fieldDeclaration 
	| constraintDeclaration 
	| methodDeclaration 
	| coverageDeclaration
	| modifierInvocation;

qualifiedBehaviorName : (actorName '.')? behaviorName;

behaviorName : Identifier;

//----------------------------------------
// actionDeclaration
actionDeclaration 
	:  'action' qualifiedBehaviorName (actionInherits)? 
	((':' NEWLINE INDENT (scenarioMemberDecl | behaviorSpecification)+ DEDENT) | NEWLINE);

actionInherits : 'inherits' qualifiedBehaviorName (inheritsCondition)? ;

//----------------------------------------
// modifierDeclaration
modifierDeclaration 
	: 'modifier' (actorName '.')? modifierName ('of'  qualifiedBehaviorName)?
	((':' NEWLINE INDENT scenarioMemberDecl+ DEDENT) | NEWLINE);

modifierName : Identifier;

//----------------------------------------
// typeExtension
typeExtension : enumTypeExtension | structuredTypeExtension;

/* 
enumTypeExtension : 'extend' enumName ':' NEWLINE INDENT
	(enumMemberDecl NEWLINE)+ DEDENT;
*/
enumTypeExtension : 'extend' enumName ':' OPEN_BRACK  enumMemberDecl (','  enumMemberDecl)* CLOSE_BRACK NEWLINE;


structuredTypeExtension : 'extend' extendableTypeName 
	':' NEWLINE INDENT extensionMemberDecl+ DEDENT;


extendableTypeName 
	: typeName 
	| qualifiedBehaviorName;

extensionMemberDecl 
	: structMemberDecl 
	| actorMemberDecl 
	| scenarioMemberDecl
	| behaviorSpecification;

//----------------------------------------
// globalParameterDeclaration
globalParameterDeclaration : 'global' fieldName (',' fieldName)* ':' typeDeclarator ('=' defaultValue)? (parameterWithDeclaration | NEWLINE);

//Type declarations
typeDeclarator : nonAggregateTypeDeclarator | aggregateTypeDeclarator;

nonAggregateTypeDeclarator : primitiveType | typeName | qualifiedBehaviorName;

aggregateTypeDeclarator : listTypeDeclarator;

listTypeDeclarator : 'list' 'of' nonAggregateTypeDeclarator;

primitiveType : 'int' | 'uint' | 'float' | 'bool' | 'string';

typeName : Identifier;

// Structured type members

// eventDeclaration
eventDeclaration 
	: 'event' eventName 
	(OPEN_PAREN argumentListSpecification CLOSE_PAREN)? 
	('is' eventSpecification)? NEWLINE;

eventSpecification 
	: eventReference (( eventFieldDecl )? 'if' eventCondition) ?
	| eventCondition ;

eventReference : '@' eventPath;
eventFieldDecl : 'as' eventFieldName;
eventFieldName : Identifier;
eventName : Identifier;
eventPath : (expression '.')? eventName;

eventCondition 
	:  boolExpression 
	|  riseExpression 
	|  fallExpression
	|  elapsedExpression 
	|  everyExpression;

riseExpression : 'rise' OPEN_PAREN boolExpression CLOSE_PAREN;
fallExpression :'fall' OPEN_PAREN boolExpression CLOSE_PAREN;
elapsedExpression : 'elapsed' OPEN_PAREN durationExpression CLOSE_PAREN;

//everyExpression : 'every' OPEN_PAREN durationExpression (',' 'offset' ':' durationExpression)? CLOSE_PAREN;
everyExpression : 'every' OPEN_PAREN durationExpression (',' Identifier{ 
offsetName = $Identifier.text
if( not (offsetName == "offset") ):
    print("%s must be offset" %offsetName)
    raise NoViableAltException(self)
} ':' durationExpression)? CLOSE_PAREN;

boolExpression : expression;
durationExpression : expression;

// fieldDeclaration
fieldDeclaration 
	:  parameterDeclaration 
	|  variableDeclaration;

//parameter-declaration ::= field-name (',' field-name)* ':' type-declarator ['=' default-value] [parameter-with-declaration] NEWLINE
//[improvement:] parameterWithDeclaration? NEWLINE -> (parameterWithDeclaration | NEWLINE)
parameterDeclaration 
	: fieldName (',' fieldName)* ':' typeDeclarator ('=' defaultValue)? (parameterWithDeclaration | NEWLINE); 

variableDeclaration 
	: 'var' fieldName (',' fieldName)* ':' typeDeclarator ('=' (sampleExpression | valueExp) )? NEWLINE;

sampleExpression 
	: 'sample' OPEN_PAREN expression ',' eventSpecification (',' defaultValue)? CLOSE_PAREN;

defaultValue : expression;

parameterWithDeclaration : 'with' ':' NEWLINE INDENT
	parameterWithMember+ DEDENT;

// add coverageDeclaration
parameterWithMember : constraintDeclaration | coverageDeclaration;

// constraintDeclaration
constraintDeclaration 
    : keepConstraintDeclaration | removeDefaultDeclaration;

keepConstraintDeclaration
    : 'keep' OPEN_PAREN (constraintQualifier)? constraintExpression CLOSE_PAREN NEWLINE;

constraintQualifier
    : 'default' | 'hard';

constraintExpression : expression;

removeDefaultDeclaration : 'remove_default' OPEN_PAREN parameterReference CLOSE_PAREN NEWLINE;

parameterReference : fieldName | fieldAccess ;

modifierInvocation : ((behaviorExpression | actorExpression) '.')? modifierName OPEN_PAREN (argumentList)? CLOSE_PAREN NEWLINE;

behaviorExpression : (actorExpression '.') behaviorName;

// behaviorSpecification
behaviorSpecification : onDirective | doDirective;

onDirective : 'on' eventSpecification ':' NEWLINE INDENT
	onMember+ DEDENT;

onMember : callDirective | emitDirective;

doDirective : 'do' doMember;

doMember 
	: (labelName ':')?(composition 
	| behaviorInvocation 
	| waitDirective 
	| emitDirective 
	| callDirective);

// composition
composition : compositionOperator (OPEN_PAREN argumentList? CLOSE_PAREN)?':' NEWLINE INDENT
	doMember+ DEDENT (behaviorWithDeclaration)?;

compositionOperator 
	: 'serial' | 'one_of' | 'parallel';

behaviorInvocation 
	: (actorExpression '.')? behaviorName OPEN_PAREN (argumentList)? CLOSE_PAREN (behaviorWithDeclaration | NEWLINE);

behaviorWithDeclaration : 'with' ':' NEWLINE INDENT
	behaviorWithMember+ DEDENT;

behaviorWithMember : constraintDeclaration
                   | modifierInvocation
                   | untilDirective;

labelName : Identifier;

actorExpression 
	: actorName;

waitDirective 
	: 'wait'  eventSpecification NEWLINE;

emitDirective :  'emit' eventName (OPEN_PAREN argumentList CLOSE_PAREN)? NEWLINE;

callDirective : 'call'  methodInvocation NEWLINE;

untilDirective : 'until' eventSpecification NEWLINE;

methodInvocation : postfixExp OPEN_PAREN (argumentList)? CLOSE_PAREN;

methodDeclaration : 'def' methodName OPEN_PAREN (argumentListSpecification)? CLOSE_PAREN ('->' returnType)? methodImplementation NEWLINE;

returnType : typeDeclarator;

methodImplementation 
	: 'is' (methodQualifier)? ('expression' expression 
	| 'undefined'
	| 'external' structuredIdentifier OPEN_PAREN (argumentList)? CLOSE_PAREN);

methodQualifier : 'only';

methodName : Identifier;

coverageDeclaration: coverDeclaration | recordDeclaration ;

coverDeclaration : 'cover' OPEN_PAREN targetName? coverageArgumentList* CLOSE_PAREN NEWLINE;

recordDeclaration : 'record' OPEN_PAREN targetName? coverageArgumentList* CLOSE_PAREN NEWLINE;

coverageArgumentList : (',' 'expression' ':' expression) #coverageExpression
                     | (',' 'unit' ':' unitName) #coverageUnit
					 | (',' 'range' ':' rangeConstructor) #coverageRange
					 | (',' 'every' ':' valueExp) #coverageEvery
					 | (',' 'event' ':' eventName) #coverageEvent
					 | (',' namedArgument) #coverageNameArgument 
					 ;

targetName : Identifier ;

//Expressions
expression 
	: implication 
	| ternaryOpExp;

ternaryOpExp 
	: implication '?' expression ':' expression;

implication : disjunction ('=>' disjunction)*;
disjunction : conjunction ('or' conjunction)*;
conjunction : inversion ('and' inversion)*;
inversion 
	: 'not' inversion 
	| relation;

relation 
	: sumExpression #sumExp
	| relation relationalOp sumExpression #relationExp;

relationalOp : '==' | '!=' | '<' | '<=' | '>' | '>=' | 'in';

sumExpression
	: term #termExp
	| sumExpression additiveOp term #additiveExp;

additiveOp 
	: '+' 
	| '-';

term 
	: factor #factorExp
	| term multiplicativeOp factor #multiplicativeExp;

multiplicativeOp 
	: '*' 
	| '/' 
	| '%';

factor 
	: postfixExp 
	| '-' factor;

postfixExp 
	: primaryExp #primaryExpression
	| postfixExp '.' 'as' OPEN_PAREN typeDeclarator CLOSE_PAREN #castExpression
	| postfixExp '.' 'is' OPEN_PAREN typeDeclarator CLOSE_PAREN #typeTestExpression
	| postfixExp OPEN_BRACK expression CLOSE_BRACK #elementAccessExpression
	| postfixExp OPEN_PAREN (argumentList)? CLOSE_PAREN #functionApplicationExpression
	| postfixExp '.' fieldName #fieldAccessExpression ; 

fieldAccess : postfixExp '.' fieldName ;
    
primaryExp 
	: valueExp
	| 'it' 
	| Identifier
	| OPEN_PAREN expression CLOSE_PAREN;

valueExp 
	: physicalLiteral
	| FloatLiteral
	| integerLiteral
	| BoolLiteral
	| StringLiteral
	| identifierReference
	| enumValueReference
	| listConstructor
	| rangeConstructor;

listConstructor : OPEN_BRACK expression (',' expression)* CLOSE_BRACK;
rangeConstructor 
	: 'range' OPEN_PAREN expression ',' expression CLOSE_PAREN 
	| OPEN_BRACK expression '..' expression CLOSE_BRACK;

identifierReference : (fieldName '.')* fieldName ;
//Common productions
argumentListSpecification : argumentSpecification (',' argumentSpecification)*;

argumentSpecification : argumentName ':' typeDeclarator ('=' defaultValue)?;

argumentName : Identifier;

argumentList 
	: positionalArgument (',' positionalArgument)* (',' namedArgument)*
	| namedArgument (',' namedArgument)*;

positionalArgument : expression;
namedArgument : argumentName ':' expression;

physicalLiteral : (FloatLiteral | integerLiteral) unitName;

integerLiteral : UintLiteral | HexUintLiteral | IntLiteral;

//----------------------------------------
// Lexer rules

NEWLINE
 : ( {self.atStartOfInput()}?   SPACES
   | ( '\r'? '\n' | '\r' | '\f' ) SPACES?
   )
   {
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
    }
 ;

OPEN_BRACK : '[' {self.opened += 1}  ;
CLOSE_BRACK : ']' {self.opened -= 1}  ;
OPEN_PAREN : '(' {self.opened += 1}  ;
CLOSE_PAREN : ')' {self.opened -= 1}  ;
 

SKIP_
 : (SPACES | LINE_JOINING)
 ->skip
 ;

 fragment 
 SPACES
 : [ \t]+
 ;
 
fragment LINE_JOINING
 : '\\' SPACES? '\r'? '\n'
 ;

fragment 
RN
    : '\r'? '\n'
    ;

BLOCK_COMMENT
    :   '/*' .*? '*/'
        -> skip
    ;

LINE_COMMENT
    :   '#' ~[\r\n\f]*
        -> skip
    ;

StringLiteral
	:   Shortstring 
	|   Longstring
	;

fragment
Shortstring
	: ('"' ShortstringElem* '"') | ('\'' ShortstringElem* '\'');


fragment
ShortstringElem 
	:	ShortstringChar | StringEscapeSeq;

fragment
ShortstringChar
	: ~[\\'"\r\n];


fragment
Longstring 
    :   ('"""' LongstringElem* '"""') | ('\'\'\'' LongstringElem* '\'\'\'');

fragment
LongstringElem : LongstringChar | StringEscapeSeq;

fragment
LongstringChar : ~'\\';

fragment
StringEscapeSeq
	: '\\'.
	| '\\' RN // [improvement:] consider: '\r'? '\n'
	;


FloatLiteral : [+-]? Digit* '.' Digit+ ([eE] [+-]? Digit+)?;

UintLiteral : Digit+;

HexUintLiteral : '0x' HexDigit+;

IntLiteral : '-' Digit+;

BoolLiteral :
  'true' | 'false';


/*
where `id-start-char` matches all characters of the following Unicode character categories:

* Ll -- Lowercase Letter
* Lm -- Modifier Letter
* Lo -- Other Letter
* Lt -- Titlecase Letter
* Lu -- Uppercase Letter
* Nl -- Letter Number

It also matches the underscore character `_` (U+005F).

`id-char` matches all characters that `id-start-char` matches, and additionally all characters of the following Unicode character categories:

* Mc -- Spacing Combining Mark
* Mn -- Nonspacing Mark
* Nd -- Decimal Number
* Pc -- Connector Punctuations

`non-vertical-line-char` matches all Unicode characters, except the vertical line `|` (U+007C) character.
*/
Identifier : ( [A-Za-z] [A-Za-z0-9_]* ) | ( '|' (~[|])+ '|' )  ;
	
fragment
NonVerticalLineChar : ~[\u007C];

fragment
Digit
	: [0-9]
	;

fragment
HexDigit
	: [0-9A-Fa-f]
	;
