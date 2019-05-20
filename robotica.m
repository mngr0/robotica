(* ::Package:: *)

(* 
Copyright (C) 2019 Marco Negrini, Adamo Fapohunda, Giacomo Leidi, Cristian Castiglione, Filippo Bartolini. 
This program is free software: you can redistribute it and/or it under the terms of the GNU General Public License 
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version. 
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
You should have received a copy of the GNU General Public License along with this program. If not, see https://www.gnu.org/licenses. 
*)

BeginPackage["robotica`"]
Off[Replace::rep]

checkJointTable::usage = "Function used to check if the given Matrix describes a valid robot. Returns the number of joints, -1 if unvalid" 

drawAPI::usage = "Function used to draw the robot passed in input"

drawAPInoH::usage = "Function used to draw the robot passed in input without showing the H matrix"

showMatrix::usage = "Function used to show the various matrix"

showEmptyMatrix::usage = "Show matrix for geometrix transformation"

showRMatrix::usage = "Show matrix correlate a revolute joint"

showPMatrix::usage = "Show matrix correlate a prismatic joint"

Begin["`Private`"]


(* Function used to check if the given Matrix (jt_List) describes a valid robot. Returns the number of joints, -1 if unvalid *)
checkJointTable[jt_List]:=
	Module[{x3,dof},
		x3 = Dimensions[jt];
		dof= x3[[2]];
		If[ Length[jt]!= 5 || Length[x3]!=2 ,
			Return[-1];
		]
		For[ i=1,i<=dof,i++,
			If[ !isPrismatic[ jt[[1,i]] ] && !isRevolutionary[ jt[[1,i]] ],
				Return[-1];
			]
		];
		Return[dof];
	];


(* Function used for check if a joint is prismatic *)
isPrismatic[jtype_String]:=MemberQ[{"p"},jtype];


(* Function used for check if a joint is revolute *)
isRevolutionary[jtype_String]:=MemberQ[{"r"},jtype];


(* Show the user the input vector *)
drawZArrow[jr_]:=
  Line[
    {
      {{0,0,0},{0,0,2jr}},
      {{0,0,2jr},{1/32,0,3/2jr}},
      {{0,0,2jr},{-1/32,0,3/2jr}},
      {{0,0,2jr},{0,1/32,3/2jr}},
      {{0,0,2jr},{0,-1/32,3/2jr}}
    }
  ];


(* Draw the x,y,z axes arrows for a single joint *)
drawCoordAxes[jr_]:=
  {
    Thick,
    {Red,drawZArrow[jr]},
    {Blue,Rotate[drawZArrow[jr],\[Pi]/2,{0,1,0}]},
    {Green,Rotate[drawZArrow[jr],-\[Pi]/2,{1,0,0}]}
  }



(* Draw joints *)
drawJoint[r_, isPrism_]:=
  Module[
    {jr = 1/5,ar = 1/20, d=1},
    {
      Blue,
      {
        If[ isPrism,
          Cuboid[{-jr,-jr,-jr},{jr,jr,jr}],

          GeometricTransformation[
           Cylinder[ { {0,0,1/8}, {0,0,-1/8} }, 1/5 ] ,
           RotationTransform[Pi/2,{0,1,0}] 
        ]
        ]
      },

      {Opacity[0.5], Green, 
        Cuboid[{-ar,-ar,jr},{ar,ar,r}]

      },

      drawCoordAxes[.7]
    }
  ];


(* Apply the trasformation to the given parameters where dz is the z translation, and the following three parameters are the axes. *)
dhTransform[dz_, dxy_, dyz_, dxz_]=
TranslationTransform[{0,0,dz}].
RotationTransform[dxz,{0,0,1}].
RotationTransform[dyz,{0,1,0}].
RotationTransform[dxy,{1,0,0}];


Options[drawRobot] = {showH -> True, showDynamic-> True};


(* Draw the robot *)
drawRobot[dof_, jt_, l_, xy_, yz_, xz_,  OptionsPattern[]]:=
  Manipulate[

    Chop[%,10^-10]; (* Replaces approximate real numbers in 10^-10 that are close to zero by the exact integer 0. *)
    Module[

      {jr = 1/10,ar = 1/40,Td,j,i,d,txy,tyz,txz},
      d=Range[dof];
      txy=Range[dof];
      tyz=Range[dof];
      txz=Range[dof];
      (* we extract the user input from the sliders, it may either refer to a prismatic joint or a revolute joint.
        Prismatic joint: the slider modify the d value
        Revolute joint: the slider modify the txy value. we decided that a revolute joint may only modify the xy angle, according to robotics theroy  *)
      For[ i=1, i<=dof, i++,
        txy[[i]]=xy[[i]]+ If[isRevolutionary[jt[[i]]],params[[i]],0];
        tyz[[i]]=yz[[i]];
        txz[[i]]=xz[[i]];
        d[[i]]=l[[i]]+ If[isPrismatic[jt[[i]]],params[[i]],0] ;
      ];

      (* Td[i] is the Transformation Function that describes the posuition and rotation of the joint i*)
      Td[1]=dhTransform[0,txy[[1]],tyz[[1]],txz[[1]]  ];
      For[ i=2, i<=dof, i++,
        Td[i]=Td[i-1].dhTransform[d[[i-1]],txy[[i]],tyz[[i]],txz[[i]]  ];
      ];


      Graphics3D[
        {
          {
          LightBrown,
            Cylinder[{{0,0,-2/5},{0,0,-1/5-1/20}},2.2]
          },

          If[ OptionValue[showH], 
            Text[ StringForm[ "\!\(\*StyleBox[\"H\",\nFontSlant->\"Italic\"]\)=``", MatrixForm[N[Chop[ Td[dof] ] ,2]]], {0,0,-3.2} ]
          ],


          Table[
            GeometricTransformation[
              drawJoint[d[[q]], isPrismatic[ jt[[q]] ] ], Td[q]
            ],
            {q,dof}
          ]
        },

        SphericalRegion->True,
        ImageSize->600,
        Boxed->False
      ]
    ],
    {
      {params,ConstantArray[0,dof]},
      ControlType->None
    },


    Dynamic[
      Grid[
        Table[
          With[ {p=i},
            If[ isPrismatic[jt[[p]]],
              {Subscript["d",p],Slider[Dynamic[params[[p]]],{0,1,1/20},ImageSize->Small],Dynamic[params[[p]]]},
              {Subscript["\[Theta]",p],Slider[Dynamic[params[[p]]],{-\[Pi],\[Pi],\[Pi]/32},ImageSize->Small],Dynamic[params[[p]]]}
            ]
         ],
        {i,dof}
        ]
      ]
    ],
    Delimiter,
    ControlPlacement->Left,
    SaveDefinitions->False 

  ];


(* Function used to draw the robot passed in input *)
drawAPI[jointTable_List]:=
Module[{dof, jt, l, axy, ayz, axz},

  dof=checkJointTable[jointTable];

  axy=Range[dof];
  axz=Range[dof];
  ayz=Range[dof];
  l=Range[dof];
  jt=Range[dof];
  If [ dof>-1,
    For[ i=1, i<=dof, i++,
      jt[[i]] = jointTable[[1,i]];
      l[[i]]=jointTable[[2,i]];
      axy[[i]]=jointTable[[3,i]];
      ayz[[i]] = jointTable[[4,i]];
      axz[[i]] = jointTable[[5,i]];
    ];
    drawRobot[dof, jt, l, axy, ayz, axz, {showH -> True, showDynamic -> True}],

    Print["invalid robot"];
  ]
]

drawAPInoH[jointTable_List]:=
Module[{dof, jt, l, axy, ayz, axz},

  dof=checkJointTable[jointTable];

  axy=Range[dof];
  axz=Range[dof];
  ayz=Range[dof];
  l=Range[dof];
  jt=Range[dof];
  If [ dof>-1,
    For[ i=1, i<=dof, i++,
      jt[[i]] = jointTable[[1,i]];
      l[[i]]=jointTable[[2,i]];
      axy[[i]]=jointTable[[3,i]];
      ayz[[i]] = jointTable[[4,i]];
      axz[[i]] = jointTable[[5,i]];
    ];
    drawRobot[dof, jt, l, axy, ayz, axz, {showH -> False, showDynamic -> True}],

    Print["invalid robot"];
  ]
]


showEmptyMatrix[]:=
dhTransform[Subscript["d","z"],Subscript["\[Theta]","xy"],Subscript["\[Theta]","yz"],Subscript["\[Theta]","xz"] ]



showRMatrix[]:=
dhTransform[0,ToString[Subscript["\[Theta]","xy"], StandardForm],0,0 ]



showPMatrix[]:=
dhTransform[Subscript["d","z"],0,0,0 ]



showMatrix[dz_, xy_, yz_, xz_]:=
dhTransform[dz,xy,yz,xz]


End[]
EndPackage[]
