:- module( data,[	element/7,	obstacle/7] ).
% Element(IdElement,X,Y,TL,TR,BL,BR).
% TL1
element(1,6,1,0,_,_,_).
element(12,1,3,0,_,_,_).
element(6,5,4,0,_,_,_).
element(15,2,5,0,_,_,_).
element(0,7,5,0,_,_,_).
% TL2
element(1,6,11,1,_,_,_).
element(12,1,3,1,_,_,_).
element(6,6,4,1,_,_,_).
element(15,5,6,1,_,_,_).
element(0,3,7,1,_,_,_).
% TR1
element(10,9,1,_,0,_,_).
element(7,11,2,_,0,_,_).
element(4,13,5,_,0,_,_).
element(14,10,7,_,0,_,_).
% TR2
element(10,9,1,_,1,_,_).
element(7,13,5,_,1,_,_).
element(4,11,2,_,1,_,_).
element(14,14,1,_,1,_,_).
% BL1
element(9,4,9,_,_,_,0,_).
element(7,1,10,_,_,0,_).
element(4,6,13,_,_,0,_).
element(14,2,14,_,_,0,_).
% BL2
element(9,1,13,_,_,1,_).
element(7,2,14,_,_,1,_).
element(4,3,9,_,_,1,_).
element(14,1,10,_,_,1,_).
% BR1
element(13,12,9,_,_,_,0).
element(2,9,10,_,_,_,0).
element(8,14,13,_,_,_,0).
element(11,9,14,_,_,_,0).
% BR2
element(13,11,14,_,_,_,1).
element(2,12,9,_,_,_,1).
element(8,14,13,_,_,_,1).
element(11,9,12,_,_,_,1).

% Contour de la carte:


obstacle(-1,_,_,_,_,_,_).
obstacle(16,_,_,_,_,_,_).

obstacle(_,-1,_,_,_,_,_).
obstacle(_,16,_,_,_,_,_).

% TL1
obstacle(3,0,0,_,_,_,1).
obstacle(5,1,0,_,_,_,0).
obstacle(6,1,0,_,_,_,1).
obstacle(6,2,0,_,_,_,0).
obstacle(1,2,0,_,_,_,1).
obstacle(1,3,0,_,_,_,0).
obstacle(2,3,0,_,_,_,1).
obstacle(5,3,0,_,_,_,0).
obstacle(5,4,0,_,_,_,1).
obstacle(4,4,0,_,_,_,0).
obstacle(2,5,0,_,_,_,0).
obstacle(3,5,0,_,_,_,1).
obstacle(2,5,0,_,_,_,0).
obstacle(2,6,0,_,_,_,0).
obstacle(7,5,0,_,_,_,1).
obstacle(8,5,0,_,_,_,0).
obstacle(7,6,0,_,_,_,0).
obstacle(0,6,0,_,_,_,1).
obstacle(6,7,0,_,_,_,1).
obstacle(7,7,0,_,_,_,0).
obstacle(7,6,0,_,_,_,0).
% TL2
obstacle(4,0,1,_,_,_,1).
obstacle(5,0,1,_,_,_,0).
obstacle(2,1,1,_,_,_,1).
obstacle(3,1,1,_,_,_,0).
obstacle(2,1,1,_,_,_,0).
obstacle(2,2,1,_,_,_,1).
obstacle(0,3,1,_,_,_,0).
obstacle(1,3,1,_,_,_,1).
obstacle(1,3,1,_,_,_,0).
obstacle(1,4,1,_,_,_,1).
obstacle(6,3,1,_,_,_,0).
obstacle(6,4,1,_,_,_,1).
obstacle(0,4,1,_,_,_,0).
obstacle(0,5,1,_,_,_,1).
obstacle(5,4,1,_,_,_,0).
obstacle(6,4,1,_,_,_,1).
obstacle(5,5,1,_,_,_,0).
obstacle(5,6,1,_,_,_,1).
obstacle(5,6,1,_,_,_,0).
obstacle(6,6,1,_,_,_,1).
obstacle(3,7,1,_,_,_,0).
obstacle(4,7,1,_,_,_,1).
obstacle(3,7,1,_,_,_,0).
obstacle(3,8,1,_,_,_,1).
obstacle(6,7,1,_,_,_,0).
obstacle(7,7,1,_,_,_,0).
obstacle(7,6,1,_,_,_,1).
obstacle(7,7,1,_,_,_,0).
% TR1
obstacle(10,0,_,0,_,_,1).
obstacle(11,0,_,0,_,_,0).
obstacle(15,3,_,0,_,_,1).
obstacle(15,4,_,0,_,_,0).
obstacle(9,1,_,0,_,_,1).
obstacle(9,0,_,0,_,_,0).
obstacle(9,1,_,0,_,_,1).
obstacle(10,1,_,0,_,_,0).
obstacle(11,2,_,0,_,_,1).
obstacle(12,2,_,0,_,_,0).
obstacle(11,2,_,0,_,_,1).
obstacle(11,3,_,0,_,_,0).
obstacle(13,5,_,0,_,_,1).
obstacle(12,5,_,0,_,_,0).
obstacle(13,5,_,0,_,_,1).
obstacle(13,6,_,0,_,_,0).
obstacle(10,7,_,0,_,_,1).
obstacle(9,7,_,0,_,_,0).
obstacle(10,7,_,0,_,_,1).
obstacle(10,6,_,0,_,_,0).
obstacle(8,7,_,0,_,_,1).
obstacle(9,7,_,0,_,_,0).
obstacle(8,7,_,0,_,_,1).
obstacle(8,6,_,0,_,_,0).
% TR2
obstacle(9,0,_,1,_,_,1).
obstacle(10,0,_,1,_,_,0).
obstacle(15,3,_,1,_,_,1).
obstacle(15,4,_,1,_,_,0).
obstacle(14,1,_,1,_,_,1).
obstacle(13,1,_,1,_,_,0).
obstacle(14,1,_,1,_,_,1).
obstacle(14,0,_,1,_,_,0).
obstacle(11,2,_,1,_,_,1).
obstacle(10,2,_,1,_,_,0).
obstacle(11,2,_,1,_,_,1).
obstacle(11,3,_,1,_,_,0).
obstacle(13,6,_,1,_,_,1).
obstacle(14,6,_,1,_,_,0).
obstacle(13,6,_,1,_,_,1).
obstacle(13,7,_,1,_,_,0).
obstacle(10,7,_,1,_,_,1).
obstacle(10,6,_,1,_,_,0).
obstacle(10,7,_,1,_,_,1).
obstacle(11,7,_,1,_,_,0).
obstacle(8,7,_,1,_,_,1).
obstacle(9,7,_,1,_,_,0).
obstacle(8,7,_,1,_,_,1).
obstacle(8,6,_,1,_,_,0).
% BL1
obstacle(7,8,_,_,0,_,1).
obstacle(6,8,_,_,0,_,0).
obstacle(7,8,_,_,0,_,1).
obstacle(7,9,_,_,0,_,0).
obstacle(4,9,_,_,0,_,1).
obstacle(3,9,_,_,0,_,0).
obstacle(4,9,_,_,0,_,1).
obstacle(4,10,_,_,0,_,0).
obstacle(1,10,_,_,0,_,1).
obstacle(1,9,_,_,0,_,0).
obstacle(1,10,_,_,0,_,1).
obstacle(2,10,_,_,0,_,0).
obstacle(6,13,_,_,0,_,1).
obstacle(5,13,_,_,0,_,0).
obstacle(6,13,_,_,0,_,1).
obstacle(6,12,_,_,0,_,0).
obstacle(2,14,_,_,0,_,1).
obstacle(3,14,_,_,0,_,0).
obstacle(2,14,_,_,0,_,1).
obstacle(2,15,_,_,0,_,0).
obstacle(0,3,_,_,0,_,1).
obstacle(0,4,_,_,0,_,0).
obstacle(3,15,_,_,0,_,1).
obstacle(4,15,_,_,0,_,0).
% BL2
obstacle(7,8,_,_,1,_,1).
obstacle(6,8,_,_,1,_,0).
obstacle(7,8,_,_,1,_,1).
obstacle(7,9,_,_,1,_,0).
obstacle(3,9,_,_,1,_,1).
obstacle(4,9,_,_,1,_,0).
obstacle(3,9,_,_,1,_,1).
obstacle(3,10,_,_,1,_,0).
obstacle(5,11,_,_,1,_,1).
obstacle(5,10,_,_,1,_,0).
obstacle(5,11,_,_,1,_,1).
obstacle(4,11,_,_,1,_,0).
obstacle(1,13,_,_,1,_,1).
obstacle(0,13,_,_,1,_,0).
obstacle(1,13,_,_,1,_,1).
obstacle(1,14,_,_,1,_,0).
obstacle(6,14,_,_,1,_,1).
obstacle(6,13,_,_,1,_,0).
obstacle(6,14,_,_,1,_,1).
obstacle(7,14,_,_,1,_,0).
obstacle(0,2,_,_,1,_,1).
obstacle(0,3,_,_,1,_,0).
obstacle(4,15,_,_,1,_,1).
obstacle(5,15,_,_,1,_,0).
% BR1
obstacle(8,8,_,_,_,0,1).
obstacle(9,8,_,_,_,0,0).
obstacle(8,8,_,_,_,0,1).
obstacle(8,9,_,_,_,0,0).
obstacle(15,9,_,_,_,0,1).
obstacle(15,10,_,_,_,0,0).
obstacle(10,15,_,_,_,0,1).
obstacle(11,15,_,_,_,0,0).
obstacle(12,9,_,_,_,0,1).
obstacle(12,8,_,_,_,0,0).
obstacle(12,9,_,_,_,0,1).
obstacle(13,9,_,_,_,0,0).
obstacle(9,10,_,_,_,0,1).
obstacle(8,10,_,_,_,0,0).
obstacle(9,10,_,_,_,0,1).
obstacle(9,11,_,_,_,0,0).
obstacle(14,13,_,_,_,0,1).
obstacle(15,13,_,_,_,0,0).
obstacle(14,13,_,_,_,0,1).
obstacle(14,14,_,_,_,0,0).
obstacle(9,14,_,_,_,0,1).
obstacle(8,14,_,_,_,0,0).
obstacle(9,14,_,_,_,0,1).
obstacle(9,13,_,_,_,0,0).
% BR2
obstacle(8,8,_,_,_,1,1).
obstacle(9,8,_,_,_,1,0).
obstacle(8,8,_,_,_,1,1).
obstacle(8,9,_,_,_,1,0).
obstacle(15,0,_,_,_,1,1).
obstacle(15,1,_,_,_,1,0).
obstacle(13,15,_,_,_,1,1).
obstacle(14,15,_,_,_,1,0).
obstacle(12,9,_,_,_,1,1).
obstacle(12,8,_,_,_,1,0).
obstacle(12,9,_,_,_,1,1).
obstacle(13,9,_,_,_,1,0).
obstacle(9,12,_,_,_,1,1).
obstacle(9,11,_,_,_,1,0).
obstacle(9,12,_,_,_,1,1).
obstacle(8,12,_,_,_,1,0).
obstacle(14,13,_,_,_,1,1).
obstacle(13,13,_,_,_,1,0).
obstacle(14,13,_,_,_,1,1).
obstacle(14,14,_,_,_,1,0).
obstacle(11,14,_,_,_,1,1).
obstacle(12,14,_,_,_,1,0).
obstacle(11,14,_,_,_,1,1).
obstacle(11,15,_,_,_,1,0).
 