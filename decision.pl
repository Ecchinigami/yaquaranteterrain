
:- module( decision, [
	init/1,
	move/2
] ).

:- use_module( data ).

% Recupere les coordonnées du robot lorsqu'il se déplace
% Incomplet

init(_):- nb_setval(openList, []), nb_setval(closedList, []), nb_setval(le_Chemin, []), nb_setval(color,0),	nb_setval(r1,[]), nb_setval(r2,[]), nb_setval(r3,[]),nb_setval(lPath,[]).


/*
  move( +L, -ActionId )
*/


%deplacement(X,Y,1,X,Y,N, TL, TR, BL, BR, N) :- Pos is [X,Y],  nb_getval(robotsPos,temp), member(Pos,temp), !. 

deplacement(X,Y,1,X,Y,N, TL, TR, BL, BR, N) :- X1 is X+1, obstacle(X1,Y, TL, TR, BL, BR,0),!.
deplacement(X,Y,3,X,Y,N, TL, TR, BL, BR, N) :- X1 is X-1, obstacle(X1,Y, TL, TR, BL, BR,0),!.
deplacement(X,Y,2,X,Y,N, TL, TR, BL, BR, N) :- Y1 is Y+1, obstacle(X,Y1, TL, TR, BL, BR,1),!.
deplacement(X,Y,4,X,Y,N, TL, TR, BL, BR, N) :- Y1 is Y-1, obstacle(X,Y1, TL, TR, BL, BR,1),!.

deplacement(X,Y, 1, NX,NY,N, TL, TR, BL, BR,NBR) :- N1 is N+1, X1 is X+1, deplacement(X1,Y,1, NX,NY, N1, TL, TR, BL, BR,NBR).
deplacement(X,Y, 3, NX,NY,N, TL, TR, BL, BR,NBR) :- N1 is N+1, X1 is X-1, deplacement(X1,Y,3, NX,NY, N1, TL, TR, BL, BR,NBR).
deplacement(X,Y, 2, NX,NY,N, TL, TR, BL, BR,NBR) :- N1 is N+1, Y1 is Y+1, deplacement(X,Y1,2, NX,NY, N1, TL, TR, BL, BR,NBR).
deplacement(X,Y, 4, NX,NY,N, TL, TR, BL, BR,NBR) :- N1 is N+1, Y1 is Y-1, deplacement(X,Y1,4, NX,NY, N1, TL, TR, BL, BR,NBR).

deplacement(X,Y, D, NX, NY, TL, TR, BL, BR,NBR) :- deplacement(X,Y, D,NX, NY,0, TL, TR, BL, BR,NBR).




% Element(IdElement,X,Y,TL,TR,BL,BR).

%move( [_| _],[]) :- nb_delete(robotsPos).

	%Robot bleu
move([TL, TR, BL, BR, IdElement, BX, BY, GX, GY, YX, YY, RX, RY],Path) :- 
	 init(_),
	 IdElement>=1, IdElement < 5,!, 
	 nb_setval(robotsPos,[[GX, GY],[YX, YY],[RX, RY]]),
	 writef('Robot bleu X=%d, Y=%d\n',[BX, BY]),
	 element(IdElement, X, Y, TL, TR, BL, BR),
	 nb_setval(color,0),
	 selectRobot([BX,BY],[GX,GY],[YX,YY],[RX,RY],_),
	 distance([BX, BY],[X, Y],C),
	 a_star([[BX, BY],0,C,-1], [[X, Y],_,0,_],Path).
	 
	 %Robot vert
move( [TL, TR, BL, BR, IdElement, BX, BY, GX, GY, YX, YY, RX, RY],Path) :-
	init(_),
	IdElement>=5, IdElement < 9,!,
	nb_setval(robotsPos,[[BX, BY],[YX, YY],[RX, RY]]),
	writef('Robot vert\n'),
	element(IdElement, X, Y, TL, TR, BL, BR),
	nb_setval(color,1),
	selectRobot([BX,BY],[GX,GY],[YX,YY],[RX,RY],_),
	distance([GX, GY],[X, Y],C),
	a_star([[GX, GY],0,C,-1],[[X, Y],_,0,_],Path).
	
	%Robot jaune
move( [TL, TR, BL, BR, IdElement, BX, BY, GX, GY, YX, YY, RX, RY],Path) :-
	init(_),
	IdElement>=9, IdElement < 13,!,
	nb_setval(robotsPos,[[BX, BY],[GX, GY],[RX, RY]]),
	writef('Robot jaune\n'),
	element(IdElement, X, Y, TL, TR, BL, BR),
	nb_setval(color,2),
	selectRobot([BX,BY],[GX,GY],[YX,YY],[RX,RY],_),
	distance([YX, YY],[X, Y],C),
	a_star([[YX, YY],0,C,-1],[[X, Y],_,0,_],Path).
	
	%Robot rouge
move( [TL, TR, BL, BR, IdElement, BX, BY, GX, GY, YX, YY, RX, RY],Path) :-
	init(_),
	IdElement>=13, IdElement < 16,!,
	nb_setval(robotsPos,[[BX, BY],[GX, GY],[YX, YY]]),
	writef('Robot rouge\n'),
	element(IdElement, X, Y, TL, TR, BL, BR),
	nb_setval(color,3),
	selectRobot([BX,BY],[GX,GY],[YX,YY],[RX,RY],_),
	writef('X:%d Y:%d\n',[X,Y]),
	distance([RX, RY],[X, Y],C),
	a_star([[RX, RY],0,C,-1],[[X, Y],_,0,_],Path).


selectRobot([BX,BY],[GX,GY],[YX,YY],[RX,RY],[BX,BY]):- nb_getval(color,0),!,nb_setval(r1,[GX,GY]),nb_setval(r2,[YX,YY]),nb_setval(r3,[RX,RY]).
selectRobot([BX,BY],[GX,GY],[YX,YY],[RX,RY],[GX,GY]):- nb_getval(color,1),!,nb_setval(r1,[BX,BY]),nb_setval(r2,[YX,YY]),nb_setval(r3,[RX,RY]).
selectRobot([BX,BY],[GX,GY],[YX,YY],[RX,RY],[YX,YY]):- nb_getval(color,2),!,nb_setval(r1,[BX,BY]),nb_setval(r2,[GX,GY]),nb_setval(r3,[RX,RY]).
selectRobot([BX,BY],[GX,GY],[YX,YY],[RX,RY],[RX,RY]):- nb_getval(color,3),!,nb_setval(r1,[BX,BY]),nb_setval(r2,[GX,GY]),nb_setval(r3,[YX,YY]).

% distance calcul la distance manhattan entre deux points (en coordonnées x,y).

distance([],[],0).
distance([A,B],[A,B],0).
distance([A1,B1],[A2,B2],C):- C is (abs(A1-A2)+abs(B1-B2)).

% Permet de parourir la liste et d'afficher les coordoonnées X,Y

parcours_liste([X,Y], [X,Y]).
parcours_liste([X1,Y1|R], [X2,Y2]):- writef('X=%d, Y=%d\n',X1,Y1), parcours_liste(R, [X2,Y2]).


% On insère tous les Noeuds dans la list openList
		
insertAllStatesInOpenList(_,_,[]).
%[PositionRobot|A]

insertAllStatesInOpenList([PositionPere,G,_,_],[PositionBalise,_,0,_],[PositionRobot|A]):-
		
				[Xb,Yb] = PositionBalise,
				[Xp,Yp] = PositionPere,	
				[Xr, Yr] = PositionRobot,
				writef('Xr: %d | Yr: %d\n', [Xr, Yr]),

				writef('insertAllStatesInOpenList: %d/%d\n',[Xr, Yr]),
				distance([Xr,Yr],[Xb,Yb],H), 
                G1 is G+1, 
				writef('insertAllStatesInOpenList1\n'),
                nb_getval(openList,OpenList2),                 
                insererAvecTri([PositionRobot, G1, H, PositionPere],OpenList2,OpenList3),
                writef('insertAllStatesInOpenList2\n'),
				nb_setval(openList,OpenList3),
				writef('insertAllStatesInOpenList3\n'),
                insertAllStatesInOpenList([PositionPere,G,_,_],[PositionBalise,_,0,_],A).

% On insère un Noeud dans une liste en le positionnant à la bonne place (tri croissant selon le cout du Noeud)
% Utilisé avec la liste closedList
				
insererAvecTri([], A, A).
insererAvecTri(A,[],[A]):-!. 
insererAvecTri([[X1,Y1],G1,H1,P1],[[[X2,Y2],G2,H2,P2]|L],[[[X1,Y1],G1,H1,P1],[[X2,Y2],G2,H2,P2]|L]):- H1+G1 =< H2+G2, !. 
insererAvecTri([[X1,Y1],G1,H1,P1],[[[X2,Y2],G2,H2,P2]|L],[[[X2,Y2],G2,H2,P2]|L1]):- G1+H1>G2+H2, insererAvecTri([[X1,Y1],G1,H1,P1],L,L1), !. 

% Permet d'afficher le contenue de ClosedList

affiche_solution(closedList, A):- parcours_liste(closedList, A).

% On regarde si le robot est sur l'objectif. True : On affiche le contenue de closedList; False : On continue de chercher le meilleur Noeud fils
% Incomplet	

si_robot_sur_objectif([X,Y], [X,Y], openList, closedList, _, A):- !, affiche_solution(closedList, A).

% Algorithme A*

a_star([PositionBalise, 0, 0, -1], [PositionBalise,_, 0,_], []).
a_star(State1, State2, Path):-nb_getval(openList, []),!, nb_setval(openList, [State1]), a_star(State2, Path).

a_star(_,_):- nb_getval(openList, []), !, fail.
a_star([PositionBalise, _, 0, _], Path):-  getBestNodeFromOpenList([PositionBalise,_,0,_]), !, writef('Fin de a_star\n'), buildPath(Path).
a_star(State2, Path):- 
		writef('meh\n'),
        extractBestNodeFromOpenList(Noeud),
		writef('meh 2\n'),
        completerClosed(Noeud), 
        writef('meh 3\n'),
		getAllAccessibleStates(Noeud, AccessibleStatesList), 
		writef('meh 4\n'),
        insertAllStatesInOpenList(Noeud, State2, AccessibleStatesList),
        writef('meh 5\n'),
		a_star(State2, Path).
 
% Retourne le Noeud de openList avec le plus petit cout

getBestNodeFromOpenList(Noeud):- nb_getval(openList, [Noeud|_]).

completerClosed(Noeud):- nb_getval(closedList, ValeurClosed), append([Noeud], ValeurClosed, ClosedList2),nb_setval(closedList, ClosedList2).

% Meme chose mais on supprime le Noeud visité de la liste openList

extractBestNodeFromOpenList(Noeud):- nb_getval(openList,[Noeud|R]), nb_setval(openList, R).

% Retourne le chemin à suivre dans la forme donnée par le sujet
% Part du Noeud fin pour reconstruire le chemin
% Incomplet

buildPath(Path):-
	getBestNodeFromOpenList([PositionBalise,_,_,Var]), 
	buildPath(PositionBalise,Var,CheminALEnver),
	inverse(CheminALEnver, Chemin),
	nb_getval(color,C),
	solutionRetenue(Chemin,C,Path).

buildPath(_,-1,[]).	

direction([X1,_],[X2,_],R) :- X1>X2 , R is 1,!.
direction([_,Y1],[_,Y2],R) :- Y1<Y2 , R is 2,!.
direction([X1,_],[X2,_],R) :- X1<X2 , R is 3,!.
direction([_,Y1],[_,Y2],R) :- Y1>Y2 , R is 4,!.

buildPath(PositionBalise,Var1,Path):-
	direction(PositionBalise,Var1,D),
	writef('Direction %d\n',D),
	append([D],Var3,Path),
	nb_getval(closedList,ClosedList2),
	member([Var1,_,_,Var2],ClosedList2),
	buildPath(Var1,Var2,Var3).
	
	
% Retourne tous les états accessibles depuis l'état actuel puis les évalues selon leur cout.

getAllAccessibleStates([[X,Y],_,_,_], AccessibleStatesList):-
		writef('getAllAccessibleStates\n'),
		deplacement(X,Y,1,NX1,NY1,TL, TR, BL, BR,_),
        deplacement(X,Y,2,NX2,NY2,TL, TR, BL, BR,_),
        deplacement(X,Y,3,NX3,NY3,TL, TR, BL, BR,_),
        deplacement(X,Y,4,NX4,NY4,TL, TR, BL, BR,_),
        evaluation([[NX1,NY1],[NX2,NY2],[NX3,NY3],[NX4,NY4]],AccessibleStatesList)
		% ,append([A,B],AccessibleStatesList2,AccessibleStatesList),
		.

% Permet d'évaluer le meilleur successeur à partir d'un état.
% Incomplet
		
evaluation(Possibilite, AccessibleStatesList):-
		nb_getval(openList,OpenList2),
		nb_getval(closedList,ClosedList2),
		etudeDeCas(Possibilite,OpenList2,ClosedList2,AccessibleStatesList).

inverse([],[]).
inverse([A,B],[A,B]).
inverse([A,B|R],I1):-inverse(R,I2), append(I2,[A,B],I1).

solutionRetenue([],_,[]).
solutionRetenue([[A1,B1]|R],C,SR1):-append([[C,[A1,B1]],SR2],SR1),solutionRetenue(R,C,SR2),!.


etudeDeCas(L,AccessibleStatesList):-
		nb_getval(openList,Open),
		nb_getval(closedList,Closed),
		etudeDeCas(L,Open,Closed,AccessibleStatesList).
			
etudeDeCas([],_,_,[]):-!.

etudeDeCas([X|R],[],[],AccessibleStatesList):-
			append([X],AccessibleStatesList2,AccessibleStatesList),
			nb_getval(openList,OpenList2),
			nb_getval(closedList,ClosedList2),
			etudeDeCas(R,OpenList2,ClosedList2,AccessibleStatesList2),!.
			
etudeDeCas([X|R],[],[[X,_,_,_]|_],AccessibleStatesList):-
			nb_getval(closedList,ClosedList2),
			nb_getval(openList,OpenList2),
			etudeDeCas(R,OpenList2,ClosedList2,AccessibleStatesList),!.
			
etudeDeCas([X|R1],[],[_|R2],AccessibleStatesList):-
			etudeDeCas([X|R1],[],R2,AccessibleStatesList),!.
			
etudeDeCas([X|R1],[[X,_,_,_]|_],closedList,AccessibleStatesList):-
			nb_getval(openList,OpenList2),
			etudeDeCas(R1,OpenList2,closedList,AccessibleStatesList).
			
etudeDeCas([X|R1],[_|R2],closedList,AccessibleStatesList):-
			etudeDeCas([X|R1],R2,closedList,AccessibleStatesList).
