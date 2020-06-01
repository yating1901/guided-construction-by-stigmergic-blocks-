
/***********************************************************/
/***********************************************************/

#include <stdlib.h>
#include <stdio.h>

#include "clock.h"
#include "huart_controller.h"
#include "task_scheduler.h"
#include "nfc_controller.h"
#include "led_controller.h"
#include "accelerometer_controller.h"
#include <string.h>

/***********************************************************/
/***********************************************************/

#define FONT_BOLD   "\e[1m"
#define FONT_NORMAL "\e[0m"
/***********************************************************/
/***********************************************************/

#define RGB_RED_OFFSET    0
#define RGB_GREEN_OFFSET  1
#define RGB_BLUE_OFFSET   2
#define RGB_UNUSED_OFFSET 3
#define RGB_LEDS_PER_FACE 4
/***********************************************************/
/***********************************************************/
void SetAllColorsOnFace(uint8_t unRed,uint8_t unGreen, uint8_t unBlue){
   for(uint8_t unLedIdx = 0; unLedIdx < RGB_LEDS_PER_FACE; unLedIdx++) {
      CLEDController::SetBrightness(unLedIdx * RGB_LEDS_PER_FACE +
                                    RGB_RED_OFFSET, unRed);
      CLEDController::SetBrightness(unLedIdx * RGB_LEDS_PER_FACE +
                                    RGB_GREEN_OFFSET, unGreen);
      CLEDController::SetBrightness(unLedIdx * RGB_LEDS_PER_FACE +
                                    RGB_BLUE_OFFSET, unBlue);
   }
}
/***********************************************************/
/***********************************************************/
void SetAllModesOnFace(CLEDController::EMode e_mode){
   for(uint8_t unLedIdx = 0; unLedIdx < RGB_LEDS_PER_FACE; unLedIdx++) {
      CLEDController::SetMode(unLedIdx * RGB_LEDS_PER_FACE 
                              + RGB_RED_OFFSET, e_mode);
      CLEDController::SetMode(unLedIdx * RGB_LEDS_PER_FACE +
                              RGB_GREEN_OFFSET, e_mode);
      CLEDController::SetMode(unLedIdx * RGB_LEDS_PER_FACE +
                              RGB_BLUE_OFFSET, e_mode);
      CLEDController::SetMode(unLedIdx * RGB_LEDS_PER_FACE +
                              RGB_UNUSED_OFFSET, CLEDController::EMode::Off);
   }
}
/***********************************************************/
/***********************************************************/
void LightenOnFace(uint8_t un_FaceNum){
   CPortController::EPort eFace = CPortController::EPort(un_FaceNum);
   if(eFace!=CPortController::EPort::Disconnect){
      CPortController::GetInstance().Select(eFace);
      SetAllColorsOnFace(0x03,0x00,0x03);  /*Pink*/
   }
}
/***********************************************************/
/***********************************************************/
void LightenBlueOnFace(uint8_t un_FaceNum){
   CPortController::EPort eFace = CPortController::EPort(un_FaceNum);
   if(eFace!=CPortController::EPort::Disconnect){
      CPortController::GetInstance().Select(eFace);
      SetAllColorsOnFace(0x05,0x01,0x00);/*Blue*/
   }
}
/***********************************************************/
/***********************************************************/
void LightenOffFace(uint8_t un_FaceNum){
   CPortController::EPort eFace = CPortController::EPort(un_FaceNum);
   if(eFace!=CPortController::EPort::Disconnect){
      CPortController::GetInstance().Select(eFace);
      SetAllColorsOnFace(0x00,0x00,0x00);  /*Off*/
   }
}
/***********************************************************/
/***********************************************************/
uint8_t GetTopFace(){
   int16_t Coordination[3];
   uint8_t un_TopFaceIndex=0;
   CPortController::EPort eTopFace = CPortController::EPort::Disconnect;
   
   Coordination[0]=CAccelerometerController::GetInstance().GetSample().X;
   Coordination[1]=CAccelerometerController::GetInstance().GetSample().Y;
   Coordination[2]=CAccelerometerController::GetInstance().GetSample().Z;

   for(uint8_t index=0; index<3; index++){
      if(abs(Coordination[index])>10000){
         switch(index){
            case 0:
               if(Coordination[index]>0){
                  eTopFace = CPortController::EPort::North; 
               }
               else{
                  eTopFace = CPortController::EPort::South; 
               }      
               break;
            case 1:
               if(Coordination[index]>0){
                  eTopFace = CPortController::EPort::West;
               }
               else{
                  eTopFace = CPortController::EPort::East;
               }
               break;
            case 2:
               if(Coordination[index]>0){
                  eTopFace = CPortController::EPort::Top;
               }
               else{
                  eTopFace = CPortController::EPort::Bottom; 
               }
               break;
            default:
               break;
         }
      }
   }
   un_TopFaceIndex=static_cast<uint8_t>(eTopFace); 
   return un_TopFaceIndex;
}

/***********************************************************/
/***********************************************************/
struct STargetRxFunctor : CNFCController::SRxFunctor {   
   virtual void operator()(const uint8_t* pun_data, uint8_t un_length ) {
      un_Count++;
      Message = pun_data[0]; 
      if(Message == 'Q'){
         IsParent=true;
      }
      for(uint8_t un_index=0; un_index<un_length-1; un_index++){
         m_punRxBuffer[un_index]=pun_data[un_index+1]; //copy exclude 'Q'
      }
      un_NumTreeInfo=un_length-1;
   }
   void Reset(){
      IsParent=false;
      un_NumTreeInfo=0;
      un_Count=0;
      Message='E';
   }
   bool IsParent=false;
   uint8_t Message='E';
   uint8_t m_punRxBuffer[10];
   uint8_t un_NumTreeInfo=0;
   uint8_t un_Count=0;
};
/***********************************************************/
/***********************************************************/
struct SInitTxFunctor : CNFCController::STxFunctor { 
   virtual uint8_t operator()(uint8_t* pun_data, uint8_t un_length){  
      un_Count++;
      pun_data[0]=Message;   //'Q'
      for(uint8_t un_TxBufferIndex=0; un_TxBufferIndex<un_NumTreeInfo; un_TxBufferIndex++){
         if(pun_TxBuffer!=nullptr){
            pun_data[un_TxBufferIndex+1]=*pun_TxBuffer++; //copy afer 'Q'
         }
      }       
      return un_NumTreeInfo+1;   //including 'Q';
   } 
   void Reset(){
      un_Count=0;
      Message='E';
      un_NumTreeInfo=0;
      pun_TxBuffer=nullptr;
   }
   uint8_t Message='E';
   uint8_t *pun_TxBuffer;
   uint8_t un_NumTreeInfo=0;
   uint8_t un_Count=0;
};
/***********************************************************/
/***********************************************************/
struct STargetTxFunctor : CNFCController::STxFunctor {
   STargetTxFunctor(STargetRxFunctor& s_rx_counter) : RxFunctor(s_rx_counter){}
   virtual uint8_t operator()(uint8_t* pun_data, uint8_t un_length){
      un_Count++;
      if(RxFunctor.Message == 'Q'){    //could remove RxFunctor;
         Message = 'R';
         for(uint8_t un_TxBufferIndex=0; un_TxBufferIndex<un_NumTreeInfo; un_TxBufferIndex++){
            if(pun_TxBuffer!=nullptr){
               pun_data[un_TxBufferIndex+1]=*pun_TxBuffer++; //copy afer 'R'
            }
        }  
      }
      pun_data[0] = Message;    
      return un_NumTreeInfo+1;   //including 'R';
   }  
   void Reset(){
      un_Count=0;
      Message='E';
      un_NumTreeInfo=0;
      pun_TxBuffer=nullptr;
   }
   STargetRxFunctor& RxFunctor; 
   uint8_t Message='E';
   uint8_t *pun_TxBuffer;
   uint8_t un_NumTreeInfo=0;
   uint8_t un_Count=0;
};
/***********************************************************/
/***********************************************************/
struct SInitRxFunctor : CNFCController::SRxFunctor {   
   virtual void operator()(const uint8_t* pun_data, uint8_t un_length ) {
      un_Count++;
      Message = pun_data[0]; 
      if(Message == 'R' && un_Count>10){
         un_ChildConnected=1;
         HasChild=true;
         for(uint8_t un_index=0; un_index<un_length-1; un_index++){
            m_punRxBuffer[un_index]=pun_data[un_index+1]; //copy exclude 'R'
         }
      }
      un_NumTreeInfo=un_length-1;
   }
   void Reset(){
      un_Count=0;
      un_ChildConnected=0;
      HasChild=false;
      un_NumTreeInfo=0;
      Message='E';
   }
   uint8_t Message='E';
   uint8_t un_ChildConnected=0;
   bool HasChild=false;
   uint8_t m_punRxBuffer[10];
   uint8_t un_NumTreeInfo=0;
   uint8_t un_Count=0;
};

/***********************************************************/
/***********************************************************/
struct SMyUserFunctor : CTaskScheduler::SUserFunctor {
   /* user defined struct for tracking the state of each face */
   struct SFace {
      CTaskScheduler::SController& Controller;
      SInitRxFunctor RxInitiatorFunctor;
      STargetRxFunctor RxTargetFunctor;
      SInitTxFunctor TxInitiatorFunctor;
      STargetTxFunctor TxTargetFunctor;

      SFace(CTaskScheduler::SController& s_controller) : 
      Controller(s_controller),
      TxTargetFunctor(RxTargetFunctor){}
   };

   /* a collection of the faces */
   CContainer<SFace, 6> Faces;
   uint32_t LastTimestamp = 0;
   uint32_t LastDiagnosticsTimestamp = 0;
   uint32_t LastMessageTimestamp = 0;

   /* constructor */
   SMyUserFunctor(){
      /* create faces on the connected ports */
      for(CTaskScheduler::SController& s_controller : CTaskScheduler::GetInstance().GetControllers()) {
         /* create a face instance */
         SFace* psFace = Faces.Insert(s_controller);
         if(psFace != nullptr) {
            /* set up the functors */
            psFace->Controller.NFC.SetInitiatorRxFunctor(psFace->RxInitiatorFunctor);
            psFace->Controller.NFC.SetTargetRxFunctor(psFace->RxTargetFunctor);
            psFace->Controller.NFC.SetInitiatorTxFunctor(psFace->TxInitiatorFunctor);
            psFace->Controller.NFC.SetTargetTxFunctor(psFace->TxTargetFunctor);
         }
      }
      for(CTaskScheduler::SController& s_controller : CTaskScheduler::GetInstance().GetControllers()) {
         CPortController::GetInstance().Select(s_controller.Port);
         CLEDController::Init();
      }
      LedInit();
   }
    enum struct EBlockState : uint8_t {
      Idle = 0,
      Query,
      WatingForResp,
      Response,
   };

   enum struct EEventFunctor: uint8_t{
      Idle=0,
      Query,
      Response,
      Command,
   };
   enum struct EDirectedFace : uint8_t {
      Right      = 0,
      Front      = 1,
      Left       = 2,
      Parent     = 3,
      Top        = 4,
      Bottom     = 5,
      Disconnect = 8,
   };
   /***********************************************************/
   /***********************************************************/
   char FaceToChar(EDirectedFace e_face) {
      switch(e_face) {
      case EDirectedFace::Right:
         return 'R';
         break;
      case EDirectedFace::Front:
         return 'F';
         break;
      case EDirectedFace::Left:
         return 'L';
         break;
      case EDirectedFace::Parent:
         return 'P';
         break;
      case EDirectedFace::Top:
         return 'T';
         break;
      case EDirectedFace::Bottom:
         return 'B';
         break;
      default:
         return 'X';
         break;
      }
   }
   /*******Set Right=0;Front;Left;Parent;Top;Bottom;***** *****/
   /***********************************************************/
   void SetDirectedFaces(uint8_t un_ParentFace, uint8_t un_TopFaceIndex){
      uint8_t SurroundedMap[6][5]{{4,3,5,1,4},   //North=0,East,South,West,Top,Bottom     
                                  {2,4,0,5,2},            
                                  {1,5,3,4,1},
                                  {5,0,4,2,5},
                                  {0,1,2,3,0},
                                  {3,2,1,0,3}};
      uint8_t RelativeFacesMap[6]={2,3,0,1,5,4}; 

      un_DirectedFaces[static_cast<uint8_t>(EDirectedFace::Parent)] = un_ParentFace; 
      un_DirectedFaces[static_cast<uint8_t>(EDirectedFace::Front)]  = RelativeFacesMap[un_ParentFace];
      un_DirectedFaces[static_cast<uint8_t>(EDirectedFace::Top)]    = un_TopFaceIndex;
      un_DirectedFaces[static_cast<uint8_t>(EDirectedFace::Bottom)] = RelativeFacesMap[un_TopFaceIndex];
      for(uint8_t un_ParentFaceIndex=0;un_ParentFaceIndex<4;un_ParentFaceIndex++){
         if(SurroundedMap[un_TopFaceIndex][un_ParentFaceIndex]== un_ParentFace){
            un_DirectedFaces[static_cast<uint8_t>(EDirectedFace::Left)]  = SurroundedMap[un_TopFaceIndex][un_ParentFaceIndex+1];
            un_DirectedFaces[static_cast<uint8_t>(EDirectedFace::Right)] = RelativeFacesMap[un_DirectedFaces[static_cast<uint8_t>(EDirectedFace::Left)]];
            break;
         }
      }   
   }
   /***********************************************************/
   /*******Convert 'North/East' to 'Left/Front'****************/
   uint8_t ConvertFace(uint8_t un_ChosenFace){
      uint8_t un_ConversedFaceName=6;
      for(uint8_t un_FaceIndex=0;un_FaceIndex<6;un_FaceIndex++){
         if(un_DirectedFaces[un_FaceIndex]==un_ChosenFace){
            un_ConversedFaceName=un_FaceIndex;
            break;
         }
      }
      return un_ConversedFaceName;
   }
   /***********************************************************/
   /**********************************************************/
   void LedInit(){
      for(SFace& s_face : Faces) {
         CPortController::GetInstance().Select(s_face.Controller.Port);
         SetAllModesOnFace(CLEDController::EMode::PWM);
         SetAllColorsOnFace(0x00,0x03,0x03);/*Green*/
      } 
   }
   /***********************************************************/
   /**Clean all the data received/sent; Disable Initiator;*****/
   void Reset(){
      //un_NumTreeInfo=0;
      for(SFace& s_face : Faces){
         if(s_face.RxTargetFunctor.IsParent){
            s_face.RxInitiatorFunctor.Reset();
            s_face.RxTargetFunctor.Reset();
            s_face.TxInitiatorFunctor.Reset();
            s_face.TxTargetFunctor.Reset();
            //s_face.Controller.NFC.SetInitiatorPolicy(CNFCController::EInitiatorPolicy::Disable);
         }
      } 
   }
   /***********************************************************/
   /************Post-Order Response****************************/
   uint8_t PackResBlockInfo(uint8_t* un_TxBlockNodeBuffer){
      uint8_t un_FaceConnectedState=0;
      uint8_t un_NumTreeInfo=0;
      uint8_t* un_pTxBuffer=nullptr;

      un_pTxBuffer=un_TxBlockNodeBuffer;
      for(uint8_t un_BitIndex=0; un_BitIndex<6; un_BitIndex++){   //R,F,L,P,T,B.. Post order;
         for(SFace& s_face : Faces){  
            if( un_DirectedFaces[un_BitIndex] == static_cast<uint8_t>(s_face.Controller.Port)){
               if(!s_face.RxTargetFunctor.IsParent && s_face.RxInitiatorFunctor.HasChild) {   

                  un_FaceConnectedState|= 1<<un_BitIndex;
                  for(uint8_t un_index=0;un_index<s_face.RxInitiatorFunctor.un_NumTreeInfo;un_index++){
                     un_TxBlockNodeBuffer[un_NumTreeInfo++]=s_face.RxInitiatorFunctor.m_punRxBuffer[un_index]; 
                  }
                  //LightenOffFace(static_cast<uint8_t>(s_face.Controller.Port));
               }
               break;
            } 
         }
      }
      un_TxBlockNodeBuffer[un_NumTreeInfo++]=un_FaceConnectedState; //the state of Child;

      for(SFace& s_face : Faces){
         if(s_face.RxTargetFunctor.IsParent){   //Response except root blcok 
            s_face.TxTargetFunctor.Message='R';
            s_face.TxTargetFunctor.un_NumTreeInfo=un_NumTreeInfo;  //un_NumTreeInfo =starts from 0, Number of Info in the array, not index;
            s_face.TxTargetFunctor.pun_TxBuffer=un_pTxBuffer;
         }
      }  
      return un_NumTreeInfo;
   }
   
   /**************************************************************/
   /*********CountNumOfChild Function*****************************/
   uint8_t CountNumOfChild(uint8_t* Parent){
      uint8_t un_NumOfChild=0;
      uint8_t ParentInfo=0;
      
      ParentInfo= *Parent;
      for(int un_BitIndex=5;un_BitIndex>=0;un_BitIndex--){
         if((ParentInfo & (1<<un_BitIndex)) != 0){
            un_NumOfChild++;
         }
      }
      return un_NumOfChild;
   }
   /**************************************************************/
   /*********Non-Recurisive Function******************************/
   uint8_t PreOrderTraverse(uint8_t* root){
      uint8_t  un_NumOfChild=0;
      uint8_t  un_SumOfChild=0;
      uint8_t* un_pStart=nullptr;
      uint8_t* un_pEnd=nullptr;

      un_pStart=root;
      un_NumOfChild=CountNumOfChild(un_pStart);
      un_pEnd=un_pStart+un_NumOfChild;

      while(un_pStart!=un_pEnd){
         un_pStart++;
         un_NumOfChild=CountNumOfChild(un_pStart);
         un_pEnd=un_pEnd+un_NumOfChild;
      }
      un_SumOfChild=un_pEnd-root;
      return un_SumOfChild;
   }
   /***********************************************************/
   /******Split messages into each face***********************/
   void DecomposeRootedTree(uint8_t* un_RootedTree, uint8_t un_NumTreeInfo){
      uint8_t  un_SelfBlockInfo;
      uint8_t* un_pStart=nullptr;
      uint8_t  un_ChildOfChild=0;

      un_pStart= un_RootedTree;                                        //ParentNode;
      un_SelfBlockInfo=*un_RootedTree;
 
      for(int un_BitIndex=5;un_BitIndex>=0;un_BitIndex--){
         if( (un_SelfBlockInfo & (1<<un_BitIndex)) != 0 ){
            for(SFace& s_face : Faces){ 
               if(static_cast<uint8_t>(s_face.Controller.Port) == un_DirectedFaces[un_BitIndex]){
                  un_pStart++; 
                  s_face.TxInitiatorFunctor.Message='Q';
                  s_face.TxInitiatorFunctor.pun_TxBuffer=un_pStart;  
                  un_ChildOfChild=PreOrderTraverse(un_pStart);
                  if(un_ChildOfChild<un_NumTreeInfo){
                     s_face.TxInitiatorFunctor.un_NumTreeInfo=un_ChildOfChild+1; //un_NumTreeInfo = Num of children plus itself;
                  }
                  break;
               }  
            }
            un_pStart=un_pStart+un_ChildOfChild; 
            LightenOnFace(un_DirectedFaces[un_BitIndex]);             //Converted from Left/Front to North;  
         }
         else{
            //LightenOffFace(un_DirectedFaces[un_BitIndex]);
         }
      }
   }
   /***********************************************************/
   /***********************************************************/
   uint8_t PackResChildInfo(uint8_t* un_ResBlockNodeBuffer){
      uint8_t un_NumTreeInfo=0;
      for(SFace& s_face : Faces){  //N,E,S,W natural order;
         if(s_face.RxTargetFunctor.IsParent){
            un_NumTreeInfo=s_face.RxTargetFunctor.un_NumTreeInfo;
            for(uint8_t un_index=0;un_index<un_NumTreeInfo;un_index++){
               un_ResBlockNodeBuffer[un_index]=s_face.RxTargetFunctor.m_punRxBuffer[un_index]; 
            }
         }
      }
      return un_NumTreeInfo;
   }

   /***********************************************************/
   /***********************************************************/
   uint8_t un_ResBlockNodeBuffer[10]={0};
   uint8_t un_NumTreeInfo=0;
   uint8_t un_TxBlockNodeBuffer[10]={0};
   uint8_t un_NumChildInfo=0;
   uint8_t un_BlockIndex=0;
   uint32_t LastQuerystamp= 0;
   uint32_t LastResponsestamp= 0;
   bool IsRoot = true;
   EBlockState m_eBlockState=EBlockState::Idle;

   uint8_t un_TopFaceIndex=8;
   uint8_t un_ParentFace=static_cast<uint8_t>(CPortController::EPort::Disconnect);
   uint8_t un_DirectedFaces[6]={8};  //Right=0,Front,Left,Parent,Top,Bottom,Disconnect=8;

   uint8_t un_RootedTree[5]={15,0,0,0,0}; 
   uint8_t un_index=0;
   /***********************************************************/
   /***********************************************************/
   virtual void operator()(uint32_t un_timestamp) override {
      bool IsCompleted=true;
      un_TopFaceIndex=GetTopFace();

      switch (m_eBlockState)
      {  case EBlockState::Idle:
            if(IsRoot){
               un_ParentFace=static_cast<uint8_t>(CPortController::EPort::West);
               //un_ParentFace = un_TopFaceIndex;
               un_NumTreeInfo = sizeof(un_RootedTree);
               SetDirectedFaces(un_ParentFace,un_TopFaceIndex);
               DecomposeRootedTree(un_RootedTree,un_NumTreeInfo); //only root block apply rooted tree;
               m_eBlockState=EBlockState::Query; 
            }
            for(SFace& s_face : Faces){
               if(s_face.RxTargetFunctor.IsParent){
                  un_ParentFace=static_cast<uint8_t>(s_face.Controller.Port);
                  SetDirectedFaces(un_ParentFace,un_TopFaceIndex);
                  un_NumTreeInfo=PackResChildInfo(un_ResBlockNodeBuffer);//store command in un_ResBlockNodeBuffer;
                  DecomposeRootedTree(un_ResBlockNodeBuffer,un_NumTreeInfo);
                  m_eBlockState=EBlockState::Query;
               }
            }
            break;
         case EBlockState::Query:
            if(!IsRoot){
               for(SFace& s_face : Faces){
                  if(!s_face.RxTargetFunctor.IsParent){
                     s_face.TxInitiatorFunctor.Message='Q';
                     s_face.Controller.NFC.SetInitiatorPolicy(CNFCController::EInitiatorPolicy::Once);
                  }
               }
            }
            else{
               for(SFace& s_face : Faces){
                  s_face.TxInitiatorFunctor.Message='Q';
                  s_face.Controller.NFC.SetInitiatorPolicy(CNFCController::EInitiatorPolicy::Once);
               }
            }
            un_NumChildInfo=PackResBlockInfo(un_TxBlockNodeBuffer);//send state from un_TxBlockNodeBuffer;
            //Reset(); //only reset parent face Info;
            m_eBlockState=EBlockState::Idle;
            if(IsRoot){
               if(un_NumChildInfo==un_NumTreeInfo){
                  for(uint8_t un_TreeIndex=0;un_TreeIndex<un_NumTreeInfo;un_TreeIndex++){
                     IsCompleted = IsCompleted && (un_RootedTree[un_TreeIndex]==un_TxBlockNodeBuffer[un_NumChildInfo-un_TreeIndex-1]);
                  }
                  if(IsCompleted){
                     for(SFace& s_face : Faces){ 
                        LightenBlueOnFace(static_cast<uint8_t>(s_face.Controller.Port));
                     }
                  }  
               }
            }
            break;
         default:
            break;
      }  
      
      if(un_timestamp - LastMessageTimestamp > 1000) {
         CHUARTController::GetInstance().Print("[%05lu] ", un_timestamp);
         CHUARTController::GetInstance().Print("%u", static_cast<uint8_t>(m_eBlockState));
         for(SFace& s_face : Faces) {
            char pchBuffer[10];
            //snprintf(pchBuffer, sizeof pchBuffer,"%c%c/%c%c",s_face.TxInitiatorFunctor.Message,s_face.RxInitiatorFunctor.Message,s_face.RxTargetFunctor.Message,s_face.TxTargetFunctor.Message);
            snprintf(pchBuffer, sizeof pchBuffer,"%c%d/%c%d",s_face.TxInitiatorFunctor.Message,s_face.TxInitiatorFunctor.un_NumTreeInfo,s_face.RxTargetFunctor.Message,s_face.RxTargetFunctor.un_NumTreeInfo);
            if(m_eBlockState==EBlockState::Query){
               CHUARTController::GetInstance().Print("%c: %5s ", FaceToChar(EDirectedFace(ConvertFace(static_cast<uint8_t>(s_face.Controller.Port)))), pchBuffer); 
            } 
            else{
            CHUARTController::GetInstance().Print("%c: %5s ", CPortController::PortToChar(s_face.Controller.Port), pchBuffer); 
            }         
         }
         CHUARTController::GetInstance().Print("\r\n");
         LastMessageTimestamp=un_timestamp;
      }    
     
      /* print extended diagnostics on key press */
      if(CHUARTController::GetInstance().HasData()) {
         
         uint8_t unData=0;
         while(CHUARTController::GetInstance().HasData()) {
            unData=CHUARTController::GetInstance().Read();
         }
         switch (unData)
         {
            case '1':
               CHUARTController::GetInstance().Print(FONT_BOLD "[%05lu] ", un_timestamp);
               for(SFace& s_face : Faces) {
                  CHUARTController::GetInstance().Print(FONT_BOLD "%d ",s_face.RxInitiatorFunctor.un_ChildConnected);
               }
               CHUARTController::GetInstance().Print(FONT_NORMAL "\r\n");
               break;
            case '2':
               for(uint8_t un_RxBufferIndex=0;un_RxBufferIndex<un_NumTreeInfo;un_RxBufferIndex++){
                  CHUARTController::GetInstance().Print(FONT_BOLD "%d ",un_ResBlockNodeBuffer[un_RxBufferIndex]);
               }
               CHUARTController::GetInstance().Print(FONT_NORMAL "\r\n");
               break;
            case '3':
               for(uint8_t un_TxBufferIndex=0;un_TxBufferIndex<un_NumChildInfo;un_TxBufferIndex++){
                  CHUARTController::GetInstance().Print(FONT_BOLD "%d ",un_TxBlockNodeBuffer[un_TxBufferIndex]);
               }
               CHUARTController::GetInstance().Print(FONT_NORMAL "\r\n");
               break;
            case '4':
               for(int un_FaceIndex=5;un_FaceIndex>=0;un_FaceIndex--){
                  CHUARTController::GetInstance().Print(FONT_BOLD "%d ",un_DirectedFaces[un_FaceIndex]);
               }
               CHUARTController::GetInstance().Print(FONT_NORMAL "\r\n");
               break;
            case '5':
                for(SFace& s_face : Faces) {
                  char pchBuffer[10];
                  snprintf(pchBuffer, sizeof pchBuffer,"%d",static_cast<uint8_t>(s_face.Controller.NFC.m_eInitiatorPolicy));
                  CHUARTController::GetInstance().Print("%c: %5s ", CPortController::PortToChar(s_face.Controller.Port), pchBuffer);           
               }
               CHUARTController::GetInstance().Print("\r\n");
               break;
            case '6':
               for(SFace& s_face : Faces) {
                  char pchBuffer[10];
                  snprintf(pchBuffer, sizeof pchBuffer,"%d%d/%d%d",s_face.TxInitiatorFunctor.un_Count,s_face.RxInitiatorFunctor.un_Count,s_face.RxTargetFunctor.un_Count,s_face.TxTargetFunctor.un_Count);
                  CHUARTController::GetInstance().Print("%c: %5s ", CPortController::PortToChar(s_face.Controller.Port), pchBuffer);           
               }
               CHUARTController::GetInstance().Print("\r\n");
               break;
            case 'Q':
               CHUARTController::GetInstance().Print(FONT_BOLD "%c ",'Q');  
               CHUARTController::GetInstance().Print(FONT_NORMAL "\r\n");   
               IsRoot=true;
               break;
            default:
               break;
         }
         
      }
   }
};

/***********************************************************/
/***********************************************************/

int main() {
   /* Enable interrupts */
   CInterruptController::GetInstance().Enable();
   /* create an instance of the user code */
   SMyUserFunctor sMyUserFunctor;
   /* assign it to the task schedule and start infinite loop */
   CTaskScheduler::GetInstance().SetUserFunctor(sMyUserFunctor);
   CTaskScheduler::GetInstance().Execute();
   return 0;
}

/***********************************************************/
/***********************************************************/