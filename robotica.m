(* ::Package:: *)

(* 
Copyright (C) 2019 Marco Negrini, Adamo Fapohunda, Giacomo Leidi, Cristian Castiglione, Filippo Bartolini. 
This program is free software: you can redistribute it and/or it under the terms of the GNU General Public License 
as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version. 
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
You should have received a copy of the GNU General Public License along with this program. If not, see https://www.gnu.org/licenses. 

Tested with Mathematica 10.2 on Debian 9 and 11.3 and on Arch Linux.
*)

BeginPackage["robotica`"]
Off[Replace::rep]

checkJointTable::usage = "Function used to check if the given Matrix describes a valid robot. Returns the number of joints, -1 if unvalid" 

drawAPI::usage = "Function used to draw the robot passed in input"

getInput::usage = "Create DH parameter by Given DOF"


Begin["`Private`"]

getInput[dof_]:=
  Module[{JointType, LinkLenght, Angle1, Angle2, Angle3 },
    If[ IntegerQ[dof] && dof>0,
      DH= Input[ "Fill out the DH parameters:
      Note: \[Alpha] and \[Theta] should be in radians.",
      (* Grid[{{a, b, c}, {x, y^2, z^3}}, Frame -> All] *)

      ze=ConstantArray[{"r",0,0,0,0},{dof}];
      k={ JointType, LinkLenght, Angle1, Angle2, Angle3 };
      b=Join[{k},ze];
      cc=Transpose[b];
      k=Join[{JointType},Array[#&,dof]];
      l = Join[{k},cc];
      Q=Transpose[l];
      Grid[ Q ,Frame->All, Alignment->Center,Background->{{Gray},{Gray},Automatic},ItemStyle->{{Directive[White,Bold,20]},{Directive [ White,Bold,20] }} ]
      ],
      Print["DOF should be a positive Integer"];
      Return[] 

    ];
  ];


(* Function used to check if the given Matrix (jt_List) describes a valid robot. Returns the number of joints, -1 if unvalid *)
checkJointTable[jt_List]:=
	Module[{dof},
        (* DoF = Degree of Freedom *)
		dof= Dimensions[jt][[2]];
		If[ Length[jt]!= 5 || Length[Dimensions[jt]]!=2 ,
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


(* Draw the x,y,z axes (purple, cyan and orange, respectively) arrows for every single joint *)
drawCoordAxes[jr_]:=
  {
    Thick,
    {Purple,drawZArrow[jr]},
    {Cyan,Rotate[drawZArrow[jr],\[Pi]/2,{0,1,0}]},
    {Orange,Rotate[drawZArrow[jr],-\[Pi]/2,{1,0,0}]}
  }


(* Draw joints *)
drawJoint[r_, isPrism_]:=
  Module[
    {jr = 1/5,ar = 1/20, d=1},
    {
      Blue,
      {
        (* If joint is prismatic, draw a blue cube *)
        If[ isPrism,
          Cuboid[{-jr,-jr,-jr},{jr,jr,jr}],

          GeometricTransformation[
           (* Else is revolute, draw a blue cylinder *)
           Cylinder[ { {0,0,1/8}, {0,0,-1/8} }, 1/5 ] ,
           RotationTransform[Pi/2,{0,1,0}] 
        ]
        ]
      },
      (* Draw a green link *)
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


(* Simple list with some options: show the position and rotation inside every image, show the dynamic control and their placement and choose the image size *)
Options[drawRobotTest] = {showPos -> True, showDynamic -> True, controlPlacement -> Left, imageSize -> 600};


(* Simple list with some options: show the position and rotation inside every image, show the dynamic control and their placement and choose the image size *)
Options[drawAPI] = {showPos->True, showDynamic->True, controlPlacement->Left, imageSize->600};

drawAPI[jointTable_List, OptionsPattern[]]:=
Module[{dof, jt, l, axy, ayz, axz},

  (* DoF = Degree of Freedom *)
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
    (* Function used to draw the robot passed in input *)
    drawRobotTest[dof, jt, l, axy, ayz, axz, {showPos->OptionValue[showPos], showDynamic->OptionValue[showDynamic], controlPlacement->OptionValue[controlPlacement], imageSize->OptionValue[imageSize]}],

    Print["invalid robot"];
  ]
]


(* Return orientation for x , y, z, angle, as List *)
extractOrientation[gt_TransformationFunction]:=
  Module[
  {res,tx,ty,tz,genericM,mgt,rus,tzgenericM,ris,txtzgenericM,x,y,z},

  genericM=Take[TransformationMatrix[
            RotationTransform[ty,{0,1,0}].
            RotationTransform[tz,{0,0,1}].
            RotationTransform[tx,{1,0,0}]
          ],{1,3},{1,3}];
  mgt=Take[TransformationMatrix[gt],{1,3},{1,3}];
  res=NSolve[
    {
      Rationalize[
        mgt[[2,1]]
        ==
        genericM[[2,1]]
      ],
      - Pi< tz<= Pi
    },
    {tz},
    Reals
  ];
  For[i=1,i<=Length[res],i++,
    tzgenericM=Chop[genericM/.res[[i]]];
    z=tz/.res[[i]];
    rus=NSolve[
      {
        Rationalize[
          mgt[[2]]
          ==
          tzgenericM[[2]]
        ],
        -Pi< tx<= Pi
      },
      {tx},
      Reals
    ];
    For[j=1,j<=Length[rus],j++,
    x=tx/.rus[[j]];
    txtzgenericM=Chop[tzgenericM/.rus[[j]]];
    ris=NSolve[
      {
        Rationalize[
          mgt[[1]]
          ==
          txtzgenericM[[1]]
        ],
        - Pi< ty<= Pi
      },
      {ty},
      Reals
    ];    

      If[Length[ris]>0, 
        y=ty/.ris[[1]];
        Return[{x,y,z}]];
    ];

  ];
  Return[{0,0,0}]
]


(* Return the manipolator position *)
extractPosition[gt_TransformationFunction]:={N[TransformationMatrix[gt][[1,4]],2],   N[TransformationMatrix[gt][[2,4]],2], N[ TransformationMatrix[gt][[3,4]],2] }


(* Draw the robot passed in input *)
drawRobotTest[dof_, jt_, l_, xy_, yz_, xz_,  OptionsPattern[]]:=
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

          If[ OptionValue[showPos], 
            Text[Style[ StringForm[ "Position = ``", extractPosition[ Td[dof].TranslationTransform[ {0,0,d[[dof]]} ] ]], FontSize -> 18], {0,0,-4.2}]
          ],
          If[ OptionValue[showPos], 
            Text[Style[ StringForm[ "Orientation = ``", extractOrientation[ Td[dof].TranslationTransform[ {0,0,d[[dof]]} ] ]], FontSize -> 18], {0,0,-5.2}]
          ],

          Table[
            GeometricTransformation[
              drawJoint[d[[q]], isPrismatic[ jt[[q]] ] ], Td[q]
            ],
            {q,dof}
          ]
        },
        SphericalRegion->True,
        ImageSize->OptionValue[imageSize],
        Boxed->False
      ]
    ],
    {
      {params,ConstantArray[0,dof]},
      ControlType->None
    }, 
             
    Dynamic@If[ OptionValue[showDynamic],
        Grid[
          Table[
            With[ {p=i},
              If[ isPrismatic[jt[[p]]],
                {Subscript["d",p],Slider[Dynamic[params[[p]]],{0,1,1/20},ImageSize->Small],Dynamic[params[[p]]]},
                {Subscript["\[Theta]",p],Slider[Dynamic[params[[p]]],{-\[Pi],\[Pi],\[Pi]/32},ImageSize->Small],Dynamic[params[[p]]]}
              ]
          ],
          {i,dof}showH
          ]
        ],Invisible@Grid[]
      ],

    Delimiter,
    ControlPlacement->OptionValue[controlPlacement],
    SaveDefinitions->False 

  ];


End[]
EndPackage[]
