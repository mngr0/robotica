(* ::Package:: *)


BeginPackage["robotica`"]
Off[Replace::rep]


TPrint::usage = "TPrint[name_String:''] prints all T matrices to the
file 'name', or to screen if no name is given."

APrint::usage = "APrint[name_String:''] prints all A matrices to the
file 'name', or to screen if no name is given."

MPrint::usage = "MPrint[M_List, text_String, name_String:''] prints the
matrix/vector M in standard form with 'text' as a label.  Saved in
file 'name' if specified."

dhInput::usage = "dhInput[] lets the user enter the DH parameters, in a list of {joint_type,r,alpha,d,theta}."

readJointTable::usage = "create DH parameter by Given DOF"
loadRobot::usage = " Create DH Parameter Table by given DH Matrix"
checkJointTable::usage = "do things" 


FKin::usage = "FKin[d_List,Theta_List] calculates the end position based on the R_List and Alpha_List preloaded and the d_List and Theta_List given by parameter"



dhTransform::usage="tmp"

drawZArrow::usage = ""
drawCoordAxes::usage = ""
drawJoint::usage = ""
drawShaft::usage = ""
drawRobot::usage = "drawRobot[] displays a manipulate window and the robot so users can adjust on joint parameters.
Optional parameters:
showArrows displays the coordinate axes,
showH writes the homogenous transform,
showManipEllipse-> False,
showPlanes displays a controller to show the xy plane at each axis (useful for inverse kinematics)
"

isPrismatic::usage= ""
drawAPI::usage = "drawAPI"
isRevolutionary::usage= ""

Begin["`Private`"]

Print["hello"];
(*
show to the user a table to fill with the values
*)
readJointTable[]:=
  Do[
		(*ask the user for
				joint number
				joints Type
				joint connections (a,alpha)
				(a is called r, because)

				d and theta, will be deduced by load robot

				store everything in a valid joint table


		*)
    If[ IntegerQ[dof] && dof>0,

      DH= Input[ "Fill out the DH parameters:
        Note: \[Alpha] and \[Theta] should be in radians.",
        ze=ConstantArray[{"r",0,0,0,0},{dof}];
        k={Type,r, \[Alpha],d,\[Theta]};
        b=Join[{k},ze];
        cc=Transpose[b];
        k=Join[{Joint},Array[#&,dof]];
        l = Join[{k},cc];
        Q=Transpose[l];
        Grid[ Q ,Frame->All, Alignment->Center,Background->{{Gray},{Gray},Automatic},ItemStyle->{{Directive[White,Bold,12]},{Directive [ White,Bold,12] }} ]
      ],

      Print["DOF should be a positive Integer"];
      Return[]

    ];


    If[ Dimensions[DH]!= {6} || DH == k ,
      Print["Cancelled"];
      Return[]
    ];

		(*call loadRobot*)
  ];


checkJointTable[jt_List]:=
	Module[{x3,dof},
		x3 = Dimensions[jt];
		dof= x3[[2]];
		If[ Length[jt]!= 3 || Length[x3]!=2 ,
			Return[-1];
		]
		For[ i=1,i<=dof,i++,
			If[ !isPrismatic[ jt[[1,i]] ] && !isRevolutionary[ jt[[1,i]] ],
				Return[-1];
				
			]
		];
		Return[dof];
	];


isPrismatic[jtype_String]:=MemberQ[{"Prismatic","prismatic","P","p"},jtype];


isRevolutionary[jtype_String]:=MemberQ[{"Revolute","revolute","R","r"},jtype];


(*
  Show the user the input vector
*)
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


drawCoordAxes[jr_]:=
  {
    Thick,
    {Red,drawZArrow[jr]},
    {Blue,Rotate[drawZArrow[jr],\[Pi]/2,{0,1,0}]},
    {Green,Rotate[drawZArrow[jr],-\[Pi]/2,{1,0,0}]}
  }


drawJoint[ j_,d_,r_,theta_,showArrow_:True]:=
  Module[
    {jr = 1/5,ar = 1/20,pr=1/7,vr=1/6},
    {
      If[ showArrow, drawCoordAxes[jr] ],
      Opacity[1],
      {
        Opacity[0.5],
        Gray,
        If[ isPrismatic[j],
          Cuboid[{-ar,-ar,-1+d-jr-.01},{ar,ar,d+.01}],

          Cylinder[ { {0,0,Min[-ar,d-jr]-.01}, {0,0,Max[ar,d]+.01}}, ar ]
        ]
      },

      {
        LightBlue,
        If[ isPrismatic[j],
          {
            Cuboid[{-jr,-jr,-jr},{jr,jr,+jr-.1}],
            Cuboid[{-jr,-jr,+jr},{jr,jr,+jr+.05}]
          },

          {
            Cylinder[{{0,0,-jr-.1},{0,0,+jr+.1}},.9*jr]
          }
        ]
      },

      Rotate[{Opacity[0.5],Gray,Cuboid[{-ar,-ar,d-ar},{r,ar,d+ar}]},theta,{0,0,1}]
    }
  ];


dhTransform[dx_, dy_, dz_, dxy_, dyz_, dxz_]:=RotationTransform[dxz,{0,0,1}].TranslationTransform[{dx,dy,dz}].RotationTransform[dxy,{1,0,0}].RotationTransform[dyz,{0,1,0}];


Options[drawRobot] = {showArrows -> True, showH -> True, showManipEllipse-> False, showPlanes->False};


drawRobot[dof_, r_, alpha_, jointtype_, OptionsPattern[]]:=
  Manipulate[

    Chop[%,10^-10];
    Module[

      {jr = 1/10,ar = 1/40,Td,j,i,d,theta},
      d=Range[6];
      theta=Range[6];
      For[ i=1,i<=dof, i++,
        If[ isPrismatic[ jointtype[[i]] ],
            theta[[i]]=0;
            d[[i]]=params[[i]],


            theta[[i]]=params[[i]];
            d[[i]]=0;
        ];
      ];
      Td[1]=RotationTransform[theta[[1]],{0,0,1}].TranslationTransform[{0,0,d[[1]] }].TranslationTransform[{r[[1]] ,0,0}].RotationTransform[alpha[[1]] ,{1,0,0}];

      For[ j=2,j<=dof,j++,

        Td[j]=Td[j-1].RotationTransform[theta[[j]],{0,0,1}].TranslationTransform[{0,0,d[[j]] }].TranslationTransform[{r[[j]] ,0,0}].RotationTransform[alpha[[j]] ,{1,0,0}];

      ];

      Graphics3D[
        {
          {
            LightBrown,
            Cylinder[{{0,0,-2/5},{0,0,-1/5-1/20}},2.2]
          },
          If[ isRevolutionary[ jointtype[[1]] ],
            drawJoint[jointtype[[1]],d[[1]],r[[1]],params[[1]],OptionValue[showArrows]],

            drawJoint[jointtype[[1]],params[[1]],r[[1]],theta[[1]]],

            OptionValue[showArrows]
          ],

          If[ dof>1,

            If[showRobot,
              Table[
                If[ isRevolutionary[ jointtype[[i]] ],
                  GeometricTransformation[
                    drawJoint[jointtype[[i]],d[[i]],r[[i]],params[[i]],OptionValue[showArrows]],
                    Td[i-1]
                  ],

                  GeometricTransformation[
                    drawJoint[jointtype[[i]],params[[i]],r[[i]],theta[[i]],OptionValue[showArrows]],
                    Td[i-1]
                  ]
                ],
                {i,2,dof}
              ]
            ]
          ],

          If[ OptionValue[showPlanes],
            GeometricTransformation[
              {
                Thick,
                {Blue,Rotate[drawZArrow[1/2],\[Pi]/2,{0,1,0}], Text[Subscript["x",planei],{.9,.2,0}]},
                {Green,Rotate[drawZArrow[1/2],-\[Pi]/2,{1,0,0}], Text[Subscript["y",planei],{.2,.9,0}]},
                Blue,
                Opacity[0.2],
                Polygon[{{-1,-1,0},{-1,1,0},{1,1,0},{1,-1,0}}]
              },

              If[planei>0,
                Td[planei],

                IdentityMatrix[4];
              ]
            ]
          ],

          If[ OptionValue[showH],
            Text[StringForm["\!\(\*StyleBox[\"H\",\nFontSlant->\"Italic\"]\)=``",MatrixForm[N[Chop[Td[dof]],2]]],{0,0,-3.2}]
          ]
        },

        SphericalRegion->True,
        ImageSize->425,
        Boxed->False
      ]
    ],
    {
      {params,ConstantArray[0,6]},
      ControlType->None
    },
    Dynamic[
      Grid[
        Table[
          With[ {p=i},
            If[ isPrismatic[jointtype[[p]]],
              {Subscript["d",p],Slider[Dynamic[params[[p]]],{0,1,1/20},ImageSize->Small],Dynamic[params[[p]]]},
              {Subscript["\[Theta]",p],Slider[Dynamic[params[[p]]],{-\[Pi],\[Pi],\[Pi]/32},ImageSize->Small],Dynamic[params[[p]]]}
              ]
          ],
          {i,dof}
        ]
      ]
    ],
    Delimiter,
    {{showRobot,True,"show robot"},{True,False}},
    {{planei,0,"xy Plane"},0,dof,1,ImageSize->Small,Appearance->"Labeled",ControlType->If[OptionValue[showPlanes],Slider,None]},
    ControlPlacement->Left,
    (* SaveDefinitions->True  *)
    SaveDefinitions->False 

  ];




drawAPI[jointTable_List]:=
Module[{r, alpha ,dof},

  dof=checkJointTable[jointTable];

  alpha=Range[dof];
  r=Range[dof];
  jointtype=Range[dof];
  If [ dof>-1,
    For[ i=1, i<=dof, i++,
      alpha[[i]]=jointTable[[3,i]];
      r[[i]]=jointTable[[2,i]];
      jointtype[[i]] = jointTable[[1,i]];
    ];
    drawRobot[dof, r, alpha, jointtype,{showArrows -> True, showH -> True, showManipEllipse-> False, showPlanes->False}],

    Print["invalid robot"];
  ]
]


End[]
EndPackage[]
