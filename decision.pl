/*


  D�finition de l'IA du joueur artificiel de Rasende Roboter


*/


:- module( decision, [
	init/1,
	move/2
] ).

:- use_module( data ).

init(_).


/*
  move( +L, -ActionId )
*/


deplacement(X,Y,1,X,Y,N, TL, TR, BL, BR, N) :- Pos is [X,Y], member(Pos, nb_getval(robotsPos)), writef('deplacement : %t\n',[N]), !. 

deplacement(X,Y,1,X,Y,N, TL, TR, BL, BR, N) :- obstacle(X,Y, TL, TR, BL, BR,0), writef('deplacement : %t\n',[N]),!.
deplacement(X,Y,3,X,Y,N, TL, TR, BL, BR, N) :- obstacle(X,Y, TL, TR, BL, BR,0), writef('deplacement : %t\n',[N]),!.
deplacement(X,Y,2,X,Y,N, TL, TR, BL, BR, N) :- obstacle(X,Y, TL, TR, BL, BR,1), writef('deplacement : %t\n',[N]),!.
deplacement(X,Y,4,X,Y,N, TL, TR, BL, BR, N) :- obstacle(X,Y, TL, TR, BL, BR,1), writef('deplacement : %t\n',[N]),!.

deplacement(X,Y, 1, NX,NY,N, TL, TR, BL, BR,NBR) :- N1 is N+1, X1 is X+1, deplacement(X1,Y,1, NX,NY, N1, TL, TR, BL, BR,NBR).
deplacement(X,Y, 3, NX,NY,N, TL, TR, BL, BR,NBR) :- N1 is N+1, X1 is X-1, deplacement(X1,Y,3, NX,NY, N1, TL, TR, BL, BR,NBR).
deplacement(X,Y, 2, NX,NY,N, TL, TR, BL, BR,NBR) :- N1 is N+1, Y1 is Y+1, deplacement(X,Y1,2, NX,NY, N1, TL, TR, BL, BR,NBR).
deplacement(X,Y, 4, NX,NY,N, TL, TR, BL, BR,NBR) :- N1 is N+1, Y1 is Y-1, deplacement(X,Y1,4, NX,NY, N1, TL, TR, BL, BR,NBR).

deplacement(X,Y, D, NX, NY, TL, TR, BL, BR,NBR) :- deplacement(X,Y, D,NX, NY,0, TL, TR, BL, BR,NBR).



move( [_| _],[]) :- nb_delete(robotsPos). 
move( [_, _, _, _, T, BX, BY, GX, GY, YX, YY, RX, RY | _],[]):- T>=1, T < 5,  nb_setval(robotsPos,[ [GX, GY],[YX, YY],[RX, RY]]).
move( [_, _, _, _, T, BX, BY, GX, GY, YX, YY, RX, RY | _],[]):- T>=5, T < 9, nb_setval(robotsPos,[[BX, BY],[YX, YY],[RX, RY]]).
move( [_, _, _, _, T, BX, BY, GX, GY, YX, YY, RX, RY | _],[]):- T>=9, T < 13, nb_setval(robotsPos,[[BX, BY],[GX, GY],[RX, RY]]).
move( [_, _, _, _, T, BX, BY, GX, GY, YX, YY, RX, RY | _],[]):- T>=13, T < 16, nb_setval(robotsPos,[[BX, BY],[GX, GY],[YX, YY]]).



move( [TL, TR, BL, BR, _, BX, BY | _],[]) :- deplacement(BX, BY, 1, NX, NY, TL, TR, BL, BR, N),   writef('Coordonnees du robot: [%t , %t] / Obstacle : [%t, %t] / Deplacement de %t case(s) \n', [BX, BY, NX, NY, N]), !.
% move( [TL, TR, BL, BR | _],[]) :- writef('TL:%t TR:%t BL:%t BR:%t \n',[TL, TR, BL, BR]), !.


/*
move( [_, _, _, _, T | _],[R, X]) :- T is 0, !, writeln('Il faut bouger nimporte quel robot'), write('\n'), random(1,4,X), random(1,4,R),  !.
move( [_, _, _, _, T | _],[0, X]) :- T > 0, T < 5, !, writeln('Il faut bouger le bleu'), write('\n'), random(1,4,X), !.
move( [_, _, _, _, T | _],[1, X]) :- T > 4, T < 9, !, writeln('Il faut bouger le vert'), write('\n'),random(1,4,X), !.
move( [_, _, _, _, T | _],[2, X]) :- T > 8, T < 13, !, writeln('Il faut bouger le jaune'), write('\n'), random(1,4,X), !.
move( [_, _, _, _, T | _],[3, X]) :- T > 12, T < 17, !, writeln('Il faut bouger le rouge'), write('\n'), random(1,4,X), !.
*/

/*
move( [_, _, _, _, _, BX, BY, GX, GY, YX, YY, RX, RY | _], [0,1]) :-
 writef("Blue:"), write(BX), writef(','), write(BY), write('\n'),
 writef("Green:"), write(GX), writef(','), write(GY), write('\n'),
 writef("Yellow:"), write(YX), writef(','), write(YY), write('\n'),
 writef("Red:"), write(RX), writef(','), write(RY), write('\n'),
 !.
*/

move( [0,0,0,0,  2, 6,1 | _], [0,1,0,4] ) :- !.
move( [0,0,0,0, 14, _,_, _,_, _,_, 5,15], [3,3,3,2,3,3,3,4] ) :- !.

move( _, [] ) :- !.
%        ^
%        |
%        Action: next configuration
