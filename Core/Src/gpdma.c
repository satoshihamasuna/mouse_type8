/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpdma.c
  * @brief   This file provides code for the configuration
  *          of the GPDMA instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "gpdma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

DMA_NodeTypeDef Node_GPDMA1_Channel1;
DMA_QListTypeDef List_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_QListTypeDef pQueueLinkList;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* GPDMA1 init function */
void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  DMA_NodeConfTypeDef NodeConfig = {0};

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel6_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel7_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  NodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
  NodeConfig.Init.Request = DMA_REQUEST_SW;
  NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  NodeConfig.Init.Direction = DMA_MEMORY_TO_MEMORY;
  NodeConfig.Init.SrcInc = DMA_SINC_INCREMENTED;
  NodeConfig.Init.DestInc = DMA_DINC_FIXED;
  NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  NodeConfig.Init.SrcBurstLength = 1;
  NodeConfig.Init.DestBurstLength = 1;
  NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  NodeConfig.Init.Mode = DMA_NORMAL;
  NodeConfig.TriggerConfig.TriggerMode = DMA_TRIGM_SINGLE_BURST_TRANSFER;
  NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_FALLING;
  NodeConfig.TriggerConfig.TriggerSelection = GPDMA1_TRIGGER_LPTIM1_CH2;
  NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  NodeConfig.SrcAddress = 0;
  NodeConfig.DstAddress = 0;
  NodeConfig.DataSize = 0;
  if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel1, NULL, &Node_GPDMA1_Channel1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel1) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel1.Instance = GPDMA1_Channel1;
  handle_GPDMA1_Channel1.InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel1.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel1.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  handle_GPDMA1_Channel1.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel1.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel1, &List_GPDMA1_Channel1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_RISING;
  if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel0, NULL, &Node_GPDMA1_Channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel0) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel0.Instance = GPDMA1_Channel0;
  handle_GPDMA1_Channel0.InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  handle_GPDMA1_Channel0.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel0.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  handle_GPDMA1_Channel0.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel0.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel0, &List_GPDMA1_Channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
